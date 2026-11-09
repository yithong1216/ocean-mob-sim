// ============================================================================
// LiDAR file formats: PCD (binary / ASCII), LAS 1.2 and LAZ (LASzip-compressed LAS)
// ============================================================================
// LAS 1.2, point data format 1: x y z (mm), intensity, return number / number of returns, classification, scan angle,
// user data, point source id, GPS time. LAZ: LASzip "pointwise chunked" compression (POINT10 v2 + GPSTIME11 v2 items,
// 50 000 points per chunk), the layout LAStools / laszip write for LAS 1.2, read by PDAL, CloudCompare, laspy, QGIS.
// Field mapping:
//   intensity       = apparent reflectance x 32768, clamped (a white Lambertian card ~32768, retroreflectors 65535)
//   classification  = ASPRS codes: 9 water (and foam), 7 noise (fog / rain / spray clutter, blooming),
//                     1 unclassified (objects); user-defined 19 person, 20 retroreflector
//   user data       = the simulator's own label (1 water 2 foam 3 person 4 retroreflector 5 object 6 clutter 7 blooming)
//   point source id = frame number;  GPS time = simulation time of the pulse (s)
//   scan angle      = horizontal angle of the beam from the sensor's forward axis (deg, clamped to +-90)
// The LASzip coder is a JavaScript port of the encoder side of LASzip, Copyright (C) Martin Isenburg / rapidlasso GmbH,
// Apache License 2.0 (arithmetic coder after Amir Said's FastAC): arithmeticencoder, arithmeticmodel,
// integercompressor, laswriteitemcompressed_v2 (POINT10, GPSTIME11) and laswritepoint (chunks and chunk table).
// The whole library is one self-contained function so the same source also runs in background workers.
const LAS_LIB_SRC = function () {
  const AC_MIN = 0x01000000;
  // ---- arithmetic encoder (one per chunk; output kept in memory, carries ripple back through it)
  class Enc {
    constructor() { this.buf = new Uint8Array(1 << 16); this.n = 0; this.base = 0; this.length = 0xFFFFFFFF; }
    put(b) {
      if (this.n === this.buf.length) { const nb = new Uint8Array(this.buf.length * 2); nb.set(this.buf); this.buf = nb; }
      this.buf[this.n++] = b;
    }
    carry() { let p = this.n - 1; while (this.buf[p] === 0xFF) { this.buf[p] = 0; p--; } this.buf[p]++; }
    renorm() {
      do { this.put(this.base >>> 24); this.base = (this.base << 8) >>> 0; this.length = (this.length << 8) >>> 0; } while (this.length < AC_MIN);
    }
    bit(m, sym) {
      const x = m.p0 * (this.length >>> 13);
      if (sym === 0) { this.length = x; m.c0++; }
      else { const ib = this.base; this.base = (this.base + x) >>> 0; this.length = (this.length - x) >>> 0; if (ib > this.base) this.carry(); }
      if (this.length < AC_MIN) this.renorm();
      if (--m.until === 0) m.update();
    }
    sym(m, s) {
      const ib = this.base;
      if (s === m.last) {
        const x = m.dist[s] * (this.length >>> 15);
        this.base = (this.base + x) >>> 0; this.length = (this.length - x) >>> 0;
      } else {
        this.length = this.length >>> 15;
        const x = m.dist[s] * this.length;
        this.base = (this.base + x) >>> 0; this.length = (m.dist[s + 1] * this.length - x) >>> 0;
      }
      if (ib > this.base) this.carry();
      if (this.length < AC_MIN) this.renorm();
      m.count[s]++;
      if (--m.until === 0) m.update();
    }
    short(s) {
      const ib = this.base;
      this.length = this.length >>> 16; this.base = (this.base + s * this.length) >>> 0;
      if (ib > this.base) this.carry();
      if (this.length < AC_MIN) this.renorm();
    }
    bits(nb, s) {
      if (nb > 19) { this.short(s & 0xFFFF); s = s >>> 16; nb -= 16; }
      const ib = this.base;
      this.length = this.length >>> nb; this.base = (this.base + s * this.length) >>> 0;
      if (ib > this.base) this.carry();
      if (this.length < AC_MIN) this.renorm();
    }
    int(s) { this.short(s & 0xFFFF); this.short(s >>> 16); }
    done() {
      const ib = this.base;
      let another = true;
      if (this.length > 2 * AC_MIN) { this.base = (this.base + AC_MIN) >>> 0; this.length = AC_MIN >>> 1; }
      else { this.base = (this.base + (AC_MIN >>> 1)) >>> 0; this.length = AC_MIN >>> 9; another = false; }
      if (ib > this.base) this.carry();
      this.renorm();
      this.put(0); this.put(0); if (another) this.put(0);
      return this.buf.subarray(0, this.n);
    }
  }
  // ---- adaptive models
  class Model {
    constructor(symbols) {
      this.symbols = symbols; this.last = symbols - 1;
      this.dist = new Uint32Array(symbols); this.count = new Uint32Array(symbols).fill(1);
      this.total = 0; this.cycle = symbols;
      this.update();
      this.until = this.cycle = (symbols + 6) >>> 1;
    }
    update() {
      if ((this.total += this.cycle) > 32768) {
        this.total = 0;
        for (let i = 0; i < this.symbols; i++) this.total += (this.count[i] = (this.count[i] + 1) >>> 1);
      }
      const scale = Math.floor(0x80000000 / this.total);
      let sum = 0;
      for (let k = 0; k < this.symbols; k++) { this.dist[k] = Math.floor(scale * sum / 65536); sum += this.count[k]; }
      this.cycle = (5 * this.cycle) >>> 2;
      const maxc = (this.symbols + 6) << 3;
      if (this.cycle > maxc) this.cycle = maxc;
      this.until = this.cycle;
    }
  }
  class Bit {
    constructor() { this.c0 = 1; this.cnt = 2; this.p0 = 1 << 12; this.cycle = this.until = 4; }
    update() {
      if ((this.cnt += this.cycle) > 8192) {
        this.cnt = (this.cnt + 1) >>> 1; this.c0 = (this.c0 + 1) >>> 1;
        if (this.c0 === this.cnt) this.cnt++;
      }
      this.p0 = Math.floor(this.c0 * Math.floor(0x80000000 / this.cnt) / 262144);
      this.cycle = (5 * this.cycle) >>> 2;
      if (this.cycle > 64) this.cycle = 64;
      this.until = this.cycle;
    }
  }
  // ---- integer compressor: corrector class k with a symbol model, then its position inside the class
  class IntC {
    constructor(enc, bits = 16, contexts = 1, bitsHigh = 8) {
      this.enc = enc; this.bitsHigh = bitsHigh; this.k = 0;
      if (bits && bits < 32) { this.cbits = bits; this.crange = 2 ** bits; this.cmin = -(this.crange / 2); this.cmax = this.cmin + this.crange - 1; }
      else { this.cbits = 32; this.crange = 0; this.cmin = -2147483648; this.cmax = 2147483647; }
      this.mBits = Array.from({ length: contexts }, () => new Model(this.cbits + 1));
      this.mCorr = [new Bit()];
      for (let i = 1; i <= this.cbits; i++) this.mCorr.push(new Model(i <= bitsHigh ? 1 << i : 1 << bitsHigh));
    }
    compress(pred, real, ctx) {
      let c = (real - pred) | 0;
      if (c < this.cmin) c += this.crange; else if (c > this.cmax) c -= this.crange;
      let c1 = c <= 0 ? -c : c - 1, k = 0;
      while (c1) { c1 = Math.floor(c1 / 2); k++; }
      this.k = k;
      this.enc.sym(this.mBits[ctx], k);
      if (k) {
        if (k < 32) {
          c = c < 0 ? c + (2 ** k - 1) : c - 1;
          if (k <= this.bitsHigh) this.enc.sym(this.mCorr[k], c);
          else {
            const k1 = k - this.bitsHigh, lo = c % (2 ** k1);
            this.enc.sym(this.mCorr[k], Math.floor(c / 2 ** k1));
            this.enc.bits(k1, lo);
          }
        }
      } else this.enc.bit(this.mCorr[0], c);
    }
  }
  class Median5 {
    constructor() { this.v = [0, 0, 0, 0, 0]; this.high = true; }
    get() { return this.v[2]; }
    add(x) {
      const v = this.v;
      if (this.high) {
        if (x < v[2]) {
          v[4] = v[3]; v[3] = v[2];
          if (x < v[0]) { v[2] = v[1]; v[1] = v[0]; v[0] = x; } else if (x < v[1]) { v[2] = v[1]; v[1] = x; } else v[2] = x;
        } else {
          if (x < v[3]) { v[4] = v[3]; v[3] = x; } else v[4] = x;
          this.high = false;
        }
      } else if (v[2] < x) {
        v[0] = v[1]; v[1] = v[2];
        if (v[4] < x) { v[2] = v[3]; v[3] = v[4]; v[4] = x; } else if (v[3] < x) { v[2] = v[3]; v[3] = x; } else v[2] = x;
      } else {
        if (v[1] < x) { v[0] = v[1]; v[1] = x; } else v[0] = x;
        this.high = true;
      }
    }
  }
  const RMAP = [[15, 14, 13, 12, 11, 10, 9, 8], [14, 0, 1, 3, 6, 10, 10, 9], [13, 1, 2, 4, 7, 11, 11, 10], [12, 3, 4, 5, 8, 12, 12, 11],
    [11, 6, 7, 8, 9, 13, 13, 12], [10, 10, 11, 12, 13, 14, 14, 13], [9, 10, 11, 12, 13, 14, 15, 14], [8, 9, 10, 11, 12, 13, 14, 15]];
  const RLEVEL = [[0, 1, 2, 3, 4, 5, 6, 7], [1, 0, 1, 2, 3, 4, 5, 6], [2, 1, 0, 1, 2, 3, 4, 5], [3, 2, 1, 0, 1, 2, 3, 4],
    [4, 3, 2, 1, 0, 1, 2, 3], [5, 4, 3, 2, 1, 0, 1, 2], [6, 5, 4, 3, 2, 1, 0, 1], [7, 6, 5, 4, 3, 2, 1, 0]];
  // ---- POINT10 v2 (the 20-byte core of LAS formats 0-5)
  class Point10 {
    constructor(enc, buf, off) {
      this.enc = enc;
      this.mChanged = new Model(64); this.icInt = new IntC(enc, 16, 4); this.mScan = [new Model(256), new Model(256)];
      this.icPsid = new IntC(enc, 16); this.mBit = []; this.mCls = []; this.mUser = [];
      this.icDx = new IntC(enc, 32, 2); this.icDy = new IntC(enc, 32, 22); this.icZ = new IntC(enc, 32, 20);
      this.lastInt = new Uint16Array(16); this.lastH = new Int32Array(8);
      this.medX = Array.from({ length: 16 }, () => new Median5()); this.medY = Array.from({ length: 16 }, () => new Median5());
      this.last = buf.slice(off, off + 20); this.ldv = new DataView(this.last.buffer);
    }
    write(buf, dv, off) {
      const e = this.enc, L = this.last, ld = this.ldv;
      const b14 = buf[off + 14], r = b14 & 7, n = (b14 >> 3) & 7, m = RMAP[n][r], l = RLEVEL[n][r];
      const inten = dv.getUint16(off + 12, true), psid = dv.getUint16(off + 18, true), lpsid = ld.getUint16(18, true);
      const ch = ((L[14] !== b14) << 5) | ((this.lastInt[m] !== inten) << 4) | ((L[15] !== buf[off + 15]) << 3) |
        ((L[16] !== buf[off + 16]) << 2) | ((L[17] !== buf[off + 17]) << 1) | (lpsid !== psid ? 1 : 0);
      e.sym(this.mChanged, ch);
      if (ch & 32) e.sym(this.mBit[L[14]] || (this.mBit[L[14]] = new Model(256)), b14);
      if (ch & 16) { this.icInt.compress(this.lastInt[m], inten, m < 3 ? m : 3); this.lastInt[m] = inten; }
      if (ch & 8) e.sym(this.mCls[L[15]] || (this.mCls[L[15]] = new Model(256)), buf[off + 15]);
      if (ch & 4) { let d = buf[off + 16] - L[16]; if (d < 0) d += 256; else if (d > 255) d -= 256; e.sym(this.mScan[(b14 >> 6) & 1], d); }
      if (ch & 2) e.sym(this.mUser[L[17]] || (this.mUser[L[17]] = new Model(256)), buf[off + 17]);
      if (ch & 1) this.icPsid.compress(lpsid, psid, 0);
      const n1 = n === 1 ? 1 : 0;
      let diff = (dv.getInt32(off, true) - ld.getInt32(0, true)) | 0;
      this.icDx.compress(this.medX[m].get(), diff, n1);
      this.medX[m].add(diff);
      let kb = this.icDx.k;
      diff = (dv.getInt32(off + 4, true) - ld.getInt32(4, true)) | 0;
      this.icDy.compress(this.medY[m].get(), diff, n1 + (kb < 20 ? kb & ~1 : 20));
      this.medY[m].add(diff);
      kb = (this.icDx.k + this.icDy.k) >>> 1;
      const z = dv.getInt32(off + 8, true);
      this.icZ.compress(this.lastH[l], z, n1 + (kb < 18 ? kb & ~1 : 18));
      this.lastH[l] = z;
      L.set(buf.subarray(off, off + 20));
    }
  }
  // ---- GPSTIME11 v2 (the double after POINT10 in formats 1 and 3)
  const fits32 = d => d >= -2147483648n && d <= 2147483647n;
  const hi32 = v => Number(BigInt.asIntN(32, BigInt.asUintN(64, v) >> 32n));
  const lo32 = v => Number(BigInt.asUintN(32, v));
  class Gps {
    constructor(enc, t0) {
      this.enc = enc; this.mMulti = new Model(516); this.m0 = new Model(6); this.ic = new IntC(enc, 32, 9);
      this.last = 0; this.next = 0; this.diff = new Int32Array(4); this.extreme = new Int32Array(4); this.t = [t0, 0n, 0n, 0n];
    }
    full(t, mdl, sym) {                    // a new time sequence: the upper 32 bits predicted, the lower 32 raw
      this.enc.sym(mdl, sym);
      this.ic.compress(hi32(this.t[this.last]), hi32(t), 8);
      this.enc.int(lo32(t));
      this.next = (this.next + 1) & 3; this.last = this.next;
      this.diff[this.last] = 0; this.extreme[this.last] = 0;
    }
    other(t, mdl, base) {                  // maybe it continues one of the other three sequences
      for (let i = 1; i < 4; i++) {
        if (fits32(t - this.t[(this.last + i) & 3])) { this.enc.sym(mdl, base + i); this.last = (this.last + i) & 3; this.write(t); return true; }
      }
      return false;
    }
    write(t) {
      const e = this.enc, a = this.last;
      if (this.diff[a] === 0) {
        if (t === this.t[a]) { e.sym(this.m0, 0); return; }
        const d64 = t - this.t[a];
        if (fits32(d64)) {
          e.sym(this.m0, 1); this.ic.compress(0, Number(d64), 0); this.diff[a] = Number(d64); this.extreme[a] = 0;
        } else {
          if (this.other(t, this.m0, 2)) return;
          this.full(t, this.m0, 2);
        }
        this.t[this.last] = t;
        return;
      }
      if (t === this.t[a]) { e.sym(this.mMulti, 511); return; }
      const d64 = t - this.t[a];
      if (fits32(d64)) {
        const d = Number(d64), ld = this.diff[a];
        const mf = Math.fround(Math.fround(d) / Math.fround(ld));
        const multi = mf >= 0 ? Math.trunc(mf + 0.5) : Math.trunc(mf - 0.5);
        const bump = () => { if (++this.extreme[a] > 3) { this.diff[a] = d; this.extreme[a] = 0; } };
        if (multi === 1) { e.sym(this.mMulti, 1); this.ic.compress(ld, d, 1); this.extreme[a] = 0; }
        else if (multi > 0) {
          if (multi < 500) { e.sym(this.mMulti, multi); this.ic.compress(Math.imul(multi, ld), d, multi < 10 ? 2 : 3); }
          else { e.sym(this.mMulti, 500); this.ic.compress(Math.imul(500, ld), d, 4); bump(); }
        } else if (multi < 0) {
          if (multi > -10) { e.sym(this.mMulti, 500 - multi); this.ic.compress(Math.imul(multi, ld), d, 5); }
          else { e.sym(this.mMulti, 510); this.ic.compress(Math.imul(-10, ld), d, 6); bump(); }
        } else { e.sym(this.mMulti, 0); this.ic.compress(0, d, 7); bump(); }
      } else {
        if (this.other(t, this.mMulti, 512)) return;
        this.full(t, this.mMulti, 512);
      }
      this.t[this.last] = t;
    }
  }
  // ---- LASzip point data: chunk-table pointer, chunks (first point raw, the rest coded), chunk table
  function lazPointData(rec, n, dataStart) {
    const CH = 50000, parts = [], bytes = [];
    const dv = new DataView(rec.buffer, rec.byteOffset, rec.byteLength);
    for (let s = 0; s < n; s += CH) {
      const e = Math.min(n, s + CH), enc = new Enc();
      const p10 = new Point10(enc, rec, s * 28), gps = new Gps(enc, dv.getBigInt64(s * 28 + 20, true));
      for (let i = s + 1; i < e; i++) { const o = i * 28; p10.write(rec, dv, o); gps.write(dv.getBigInt64(o + 20, true)); }
      const body = enc.done(), chunk = new Uint8Array(28 + body.length);
      chunk.set(rec.subarray(s * 28, s * 28 + 28)); chunk.set(body, 28);
      parts.push(chunk); bytes.push(chunk.length);
    }
    let tableBody = new Uint8Array(0);
    if (bytes.length) {
      const enc = new Enc(), ic = new IntC(enc, 32, 2);
      bytes.forEach((b, i) => ic.compress(i ? bytes[i - 1] : 0, b, 1));
      tableBody = enc.done();
    }
    const total = 8 + bytes.reduce((a, b) => a + b, 0), out = new Uint8Array(total + 8 + tableBody.length), odv = new DataView(out.buffer);
    odv.setBigInt64(0, BigInt(dataStart + total), true);
    let p = 8;
    for (const c of parts) { out.set(c, p); p += c.length; }
    odv.setUint32(p, 0, true); odv.setUint32(p + 4, bytes.length, true);
    out.set(tableBody, p + 8);
    return out;
  }
  // ---- LAS 1.2 / LAZ file from a gathered frame
  const LAS_CLASS = [1, 9, 9, 19, 20, 1, 7, 7];
  function lasFile(f, o) {
    const n = f.n, rec = new Uint8Array(n * 28), dv = new DataView(rec.buffer), S = 0.001;
    const lo = (a) => { let v = Infinity; for (let i = 0; i < n; i++) if (a[i] < v) v = a[i]; return n ? Math.floor(v) : 0; };
    const off = [lo(f.x), lo(f.y), lo(f.z)];
    const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity], byRet = [0, 0, 0, 0, 0];
    for (let i = 0; i < n; i++) {
      const p = i * 28, q = [Math.round((f.x[i] - off[0]) / S), Math.round((f.y[i] - off[1]) / S), Math.round((f.z[i] - off[2]) / S)];
      for (let a = 0; a < 3; a++) { dv.setInt32(p + a * 4, q[a], true); const v = off[a] + q[a] * S; if (v < mn[a]) mn[a] = v; if (v > mx[a]) mx[a] = v; }
      dv.setUint16(p + 12, Math.min(65535, Math.max(0, Math.round(f.refl[i] * 32768))), true);
      const r = Math.min(f.ret[i], 7), nr = Math.min(Math.max(f.nret[i], r), 7);
      rec[p + 14] = r | (nr << 3);
      rec[p + 15] = LAS_CLASS[f.label[i]] ?? 1;
      dv.setInt8(p + 16, f.angle[i]);
      rec[p + 17] = f.label[i];
      dv.setUint16(p + 18, o.pointSourceId & 0xFFFF, true);
      dv.setFloat64(p + 20, f.t[i], true);
      if (r >= 1 && r <= 5) byRet[r - 1]++;
    }
    if (!n) for (let a = 0; a < 3; a++) mn[a] = mx[a] = 0;
    const vlrLen = o.compress ? 54 + 46 : 0, start = 227 + vlrLen;
    const body = o.compress ? lazPointData(rec, n, start) : rec;
    const out = new Uint8Array(start + body.length), hv = new DataView(out.buffer);
    const str = (p, s, len) => { for (let i = 0; i < len; i++) out[p + i] = i < s.length ? s.charCodeAt(i) & 0x7f : 0; };
    const now = new Date(), doy = Math.floor((now - new Date(now.getFullYear(), 0, 0)) / 86400000);
    str(0, 'LASF', 4);
    out[24] = 1; out[25] = 2;                                         // version 1.2
    str(26, o.system || 'Ocean MOB simulator', 32);
    str(58, o.software || 'Ocean MOB simulator LiDAR', 32);
    hv.setUint16(90, doy, true); hv.setUint16(92, now.getFullYear(), true);
    hv.setUint16(94, 227, true); hv.setUint32(96, start, true); hv.setUint32(100, o.compress ? 1 : 0, true);
    out[104] = o.compress ? 1 | 128 : 1;                            // point format 1 (+128: LASzip compressed)
    hv.setUint16(105, 28, true); hv.setUint32(107, n, true);
    byRet.forEach((c, i) => hv.setUint32(111 + i * 4, c, true));
    for (let a = 0; a < 3; a++) { hv.setFloat64(131 + a * 8, S, true); hv.setFloat64(155 + a * 8, off[a], true); }
    for (let a = 0; a < 3; a++) { hv.setFloat64(179 + a * 16, mx[a], true); hv.setFloat64(187 + a * 16, mn[a], true); }
    if (o.compress) {                                                 // LASzip VLR
      const v = 227;
      str(v + 2, 'laszip encoded', 16); hv.setUint16(v + 18, 22204, true); hv.setUint16(v + 20, 46, true);
      str(v + 22, 'LASzip compression (JavaScript)', 32);
      const p = v + 54;
      hv.setUint16(p, 2, true); hv.setUint16(p + 2, 0, true);          // pointwise chunked, arithmetic coder
      out[p + 4] = 2; out[p + 5] = 2; hv.setUint16(p + 6, 0, true);     // LASzip 2.2.0
      hv.setUint32(p + 8, 0, true); hv.setUint32(p + 12, 50000, true);
      hv.setBigInt64(p + 16, -1n, true); hv.setBigInt64(p + 24, -1n, true);
      hv.setUint16(p + 32, 2, true);
      hv.setUint16(p + 34, 6, true); hv.setUint16(p + 36, 20, true); hv.setUint16(p + 38, 2, true);   // POINT10 v2
      hv.setUint16(p + 40, 7, true); hv.setUint16(p + 42, 8, true); hv.setUint16(p + 44, 2, true);    // GPSTIME11 v2
    }
    out.set(body, start);
    return out;
  }
  return { lasFile };
};
const LAS_LIB = LAS_LIB_SRC();

// ---- background encoding: LAZ costs ~1-3 us per point, so frames are coded in two workers when possible
const lasWorkers = [];
let lasJob = 0;
const lasJobs = new Map();
try {
  const src = `const LIB = (${LAS_LIB_SRC.toString()})();\nonmessage = e => { const out = LIB.lasFile(e.data.f, e.data.o); postMessage({ id: e.data.id, out }, [out.buffer]); };`;
  const url = URL.createObjectURL(new Blob([src], { type: 'text/javascript' }));
  for (let i = 0; i < 2; i++) {
    const w = new Worker(url);
    w.onmessage = e => { const j = lasJobs.get(e.data.id); lasJobs.delete(e.data.id); if (j) j.resolve(e.data.out); };
    w.onerror = e => {        // fall back to the main thread; jobs already handed to the worker are lost
      console.warn('LAS worker failed, encoding on the main thread', e.message);
      lasWorkers.length = 0;
      for (const j of lasJobs.values()) j.reject(new Error('LAS worker failed'));
      lasJobs.clear();
    };
    lasWorkers.push(w);
  }
} catch (e) { lasWorkers.length = 0; }
function lasEncode(f, o) {
  if (!lasWorkers.length) return Promise.resolve(LAS_LIB.lasFile(f, o));
  const id = ++lasJob, w = lasWorkers[id % lasWorkers.length];
  return new Promise((resolve, reject) => {
    lasJobs.set(id, { resolve, reject });
    w.postMessage({ id, f, o }, [f.x.buffer, f.y.buffer, f.z.buffer, f.refl.buffer, f.label.buffer, f.ret.buffer, f.nret.buffer, f.t.buffer, f.angle.buffer]);
  });
}

// ---------------------------------------------------------------------------- gather points from the GPU buffer
// Solid-state scanners: the pulses [s0, s0 + np) of the ring buffer (oldest first); spinning: the latest full turn.
// Each pulse owns R texels (consecutive texels, or consecutive rows when spinning), its detected echoes first.
// world = false: sensor frame, x forward, y left, z up (ROS REP-103); true: x east, y north, z up.
const _lgV = new THREE.Vector3(), _lgInv = new THREE.Matrix4();
function lidarGather(s0, np, world, tEnd) {
  const M = L.model, R = L.R, W = L.W, H = L.H;
  const spin = M.kind === 0, nPulse = spin ? W * (H / R) : np;
  let rows0 = 0, rows1 = H - 1, wrapRows = null;
  if (!spin) {
    const t0 = s0 * R, t1 = (s0 + np) * R, total = W * H;
    if (t1 <= total) { rows0 = Math.floor(t0 / W); rows1 = Math.floor((t1 - 1) / W); }
    else { rows0 = Math.floor(t0 / W); rows1 = H - 1; wrapRows = Math.floor((t1 - total - 1) / W); }
  }
  // read the rows that hold these pulses (two reads when the ring wraps)
  if (!lidarGather.buf || lidarGather.buf.length !== W * H * 4) lidarGather.buf = new Float32Array(W * H * 4);   // reused frame to frame
  const buf = lidarGather.buf;
  const read = (r0, r1) => { if (r1 >= r0) renderer.readRenderTargetPixels(lidarPtsRT, 0, r0, W, r1 - r0 + 1, buf.subarray(r0 * W * 4, (r1 + 1) * W * 4)); };
  read(rows0, rows1);
  if (wrapRows !== null) read(0, wrapRows);
  renderer.setRenderTarget(null);
  _lgInv.copy(L.mat).invert();
  const cap = nPulse * R;
  const f = { n: 0, x: new Float64Array(cap), y: new Float64Array(cap), z: new Float64Array(cap), refl: new Float32Array(cap),
    label: new Uint8Array(cap), ret: new Uint8Array(cap), nret: new Uint8Array(cap), t: new Float64Array(cap), angle: new Int8Array(cap) };
  const cols = W, chs = H / R, total = W * H;
  for (let i = 0; i < nPulse; i++) {
    let tex0, step, tp;
    if (spin) {                                    // oldest column first
      const j = Math.floor(i / chs), ch = i - j * chs, col = (L.col + j) % cols;
      tex0 = ch * R * W + col; step = W;
      tp = tEnd - (cols - 1 - j) / (cols * params.lidarHz);
    } else {
      tex0 = (((s0 + i) % L.N) * R) % total; step = 1;
      tp = tEnd - (np - 1 - i) / M.rate;
    }
    const first = f.n;
    for (let k = 0; k < R; k++) {
      const q4 = (tex0 + k * step) * 4, w = buf[q4 + 3];
      if (!(w > 0.5)) break;                        // echoes of a pulse are stored first-return first
      const q = Math.floor(w / 1000), n = f.n;
      _lgV.set(buf[q4], buf[q4 + 1], buf[q4 + 2]);
      if (world) { f.x[n] = _lgV.x; f.y[n] = -_lgV.z; f.z[n] = _lgV.y; }
      _lgV.applyMatrix4(_lgInv);                   // three.js sensor frame: x right, y up, -z forward
      if (!world) { f.x[n] = -_lgV.z; f.y[n] = -_lgV.x; f.z[n] = _lgV.y; }
      f.angle[n] = Math.max(-90, Math.min(90, Math.round(Math.atan2(_lgV.x, -_lgV.z) * R2D)));
      f.refl[n] = w - q * 1000; f.label[n] = q % 16; f.ret[n] = Math.floor(q / 16) + 1; f.t[n] = tp;
      f.n++;
    }
    for (let j = first; j < f.n; j++) f.nret[j] = f.n - first;
  }
  return f;
}
// the current integration window (solid-state) or full turn (spinning)
function lidarGatherWindow(world) {
  const M = L.model;
  if (M.kind === 0) return lidarGather(0, 0, world, simTime);
  const np = Math.min(L.N - 1, Math.round(M.rate * params.lidarInteg));
  return lidarGather((L.head - np + L.N) % L.N, np, world, simTime);
}

// ---------------------------------------------------------------------------- PCD writers
function pcdHeader(n, comment, ascii) {
  return ['# .PCD v0.7 - ' + comment.replace(/[^ -~]/g, ''), 'VERSION 0.7', 'FIELDS x y z intensity label return',
    ascii ? 'SIZE 4 4 4 4 4 4' : 'SIZE 4 4 4 4 1 1', ascii ? 'TYPE F F F F U U' : 'TYPE F F F F U U', 'COUNT 1 1 1 1 1 1',
    `WIDTH ${n}`, 'HEIGHT 1', 'VIEWPOINT 0 0 0 1 0 0 0', `POINTS ${n}`, ascii ? 'DATA ascii' : 'DATA binary', ''].join('\n');
}
function pcdBinary(f, comment) {
  const head = new TextEncoder().encode(pcdHeader(f.n, comment, false));
  const out = new Uint8Array(head.length + f.n * 18), dv = new DataView(out.buffer, head.length);
  out.set(head, 0);
  for (let i = 0; i < f.n; i++) {
    const o = i * 18;
    dv.setFloat32(o, f.x[i], true); dv.setFloat32(o + 4, f.y[i], true); dv.setFloat32(o + 8, f.z[i], true);
    dv.setFloat32(o + 12, f.refl[i], true); dv.setUint8(o + 16, f.label[i]); dv.setUint8(o + 17, f.ret[i]);
  }
  return out;
}
function pcdAscii(f, comment) {
  const lines = [pcdHeader(f.n, comment, true)];
  for (let i = 0; i < f.n; i++) lines.push(`${f.x[i].toFixed(3)} ${f.y[i].toFixed(3)} ${f.z[i].toFixed(3)} ${f.refl[i].toFixed(4)} ${f.label[i]} ${f.ret[i]}`);
  return lines.join('\n') + '\n';
}
// one call for the three formats: resolves to a Uint8Array (PCD: binary unless ascii)
function lidarEncode(f, fmt, comment, frameNo = 0, ascii = false) {
  if (fmt === 'las' || fmt === 'laz') {
    return lasEncode(f, { compress: fmt === 'laz', pointSourceId: frameNo, software: 'Ocean MOB sim LiDAR ' + (L.model.short || '').replace(/[^ -~]/g, '') });
  }
  return Promise.resolve(ascii ? new TextEncoder().encode(pcdAscii(f, comment)) : pcdBinary(f, comment));
}
