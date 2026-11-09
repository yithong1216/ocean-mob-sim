// ============================================================================
// Language: English / 简体中文
// ============================================================================
// UI text is written in English in the code and passed through i18n(); ZH holds the Chinese. The control panel
// is translated in place by features/zz_language.js (which also draws the EN / 中文 switch). A key of the form
// 'Folder>Label', 'opt:Control>Option' or 'tag>word' gives a label its own wording in one place only.
// The choice is remembered per browser; the first visit follows the browser language.
const LANG_KEY = 'oceanMob.lang';
let LANG = (() => {
  try { const s = localStorage.getItem(LANG_KEY); if (s === 'en' || s === 'zh') return s; } catch (e) { /* storage blocked */ }
  return /^zh/i.test(navigator.language || '') ? 'zh' : 'en';
})();
const LANG_HOOKS = [];          // refresh functions for text that is only rebuilt on change
const I18N_GUIS = [];           // extra lil-gui panels to translate (the main one is always included)
function i18n(s, ctx) {
  if (LANG !== 'zh' || s == null) return s;
  if (ctx !== undefined) { const k = ctx + '>' + s; if (k in ZH) return ZH[k]; }
  return ZH[s] ?? s;
}
const ZH = {
  // ---- page, HUD, loading
  'Ocean MOB Simulator': '海上落水人员仿真',
  'Ocean · persons in water': '海面 · 落水人员',
  'Drag to orbit · wheel to zoom · right-drag to pan': '拖动旋转 · 滚轮缩放 · 右键拖动平移',
  'Sea state': '海况', 'Wind': '风', 'from': '来向', 'drift': '漂流', 'Sun': '太阳高度', 'sim t': '仿真时间',
  'In water': '落水人数', ' (loading models…)': '（模型加载中…）', 'mean submerged': '平均入水', 'group': '人群中心',
  'compass>N': '北', 'compass>NE': '东北', 'compass>E': '东', 'compass>SE': '东南',
  'compass>S': '南', 'compass>SW': '西南', 'compass>W': '西', 'compass>NW': '西北',
  'Loading ocean and people…': '正在加载海面与人员…',
  'Could not load the human models from the CDN (check the network connection).': '无法从 CDN 加载人体模型（请检查网络连接）。',
  // ---- sensor windows
  'EO · RGB': '可见光 · RGB', '● REC': '● 录制', 'LWIR 8–14 µm': '长波红外 8–14 µm',
  'WHITE-HOT': '白热', 'BLACK-HOT': '黑热', 'IRONBOW': '铁红', 'RAINBOW': '彩虹', 'AGC …': '自动增益 …',
  'LiDAR': '激光雷达',
  'ROSETTE 70°': '花瓣扫描 70°', 'L2 NON-REP 70°×75°': 'L2 非重复 70°×75°', 'L2 REP 70°×3°': 'L2 重复 70°×3°',
  'RETURN': '回波', 'ret>1st': '第1', 'ret>2nd': '第2', 'ret>3rd': '第3', 'ret>4th': '第4', 'ret>5th': '第5',
  'RGB from the EO camera · grey outside its view': 'RGB 取自可见光相机 · 视场外为灰色', 'ROSETTE 15°': '花瓣扫描 15°', '32-CH 360°': '32 线 360°', '64-CH 360°': '64 线 360°', '128-CH 360°': '128 线 360°',
  '3D CHASE': '3D 跟随', 'SENSOR POV': '传感器视角', 'BIRD\'S-EYE': '鸟瞰',
  'REFL': '反射率', '100% · retro white': '100% · 白 = 反光', 'RANGE': '距离', 'HEIGHT': '高度',
  'sem>water': '水面', 'sem>foam': '泡沫', 'sem>person': '人员', 'sem>retro': '反光', 'sem>object': '物体', 'sem>clutter': '杂波',
  'Drag to orbit · wheel to zoom · double-click to reset': '拖动旋转 · 滚轮缩放 · 双击复位',

  // ---- sea condition (0-9 scale, 中国海况等级)
  'Sea condition': '海况等级',
  'sea>Calm (glassy)': '无浪', 'sea>Calm (rippled)': '微浪', 'sea>Smooth (wavelets)': '小浪', 'sea>Slight': '轻浪',
  'sea>Moderate': '中浪', 'sea>Rough': '大浪', 'sea>Very rough': '巨浪', 'sea>High': '狂浪', 'sea>Very high': '狂涛',
  'sea>Phenomenal': '怒涛',
  '0 · Calm (glassy) (0 m)': '0 级 · 无浪（0 m）',
  '1 · Calm (rippled) (0–0.1 m)': '1 级 · 微浪（0–0.1 m）',
  '2 · Smooth (wavelets) (0.1–0.5 m)': '2 级 · 小浪（0.1–0.5 m）',
  '3 · Slight (0.5–1.25 m)': '3 级 · 轻浪（0.5–1.25 m）',
  '4 · Moderate (1.25–2.5 m)': '4 级 · 中浪（1.25–2.5 m）',
  '5 · Rough (2.5–4 m)': '5 级 · 大浪（2.5–4 m）',
  '6 · Very rough (4–6 m)': '6 级 · 巨浪（4–6 m）',
  '7 · High (6–9 m)': '7 级 · 狂浪（6–9 m）',
  '8 · Very high (9–14 m)': '8 级 · 狂涛（9–14 m）',
  '9 · Phenomenal (> 14 m)': '9 级 · 怒涛（> 14 m）',
  'The sea is like a mirror, or only swell is present.': '海面光滑如镜，或仅有涌浪存在。',
  'Ripples, or ripples together with swell.': '波纹，或涌浪和波纹同时存在。',
  'Small wavelets; crests begin to break, but the foam looks glassy rather than white.': '波浪很小，波峰开始破裂，浪花不显白色而呈玻璃色。',
  'Waves are small but conspicuous; crests break, with white foam (whitecaps) in places.': '波浪不大，但很触目，波峰破裂，其中有些地方形成白色浪花——白浪。',
  'Waves take a pronounced form, with whitecaps everywhere.': '波浪具有明显的形状，到处形成白浪。',
  'Large crests form; foam covers much of them and the wind starts to blow it off the crests.': '出现高大的波峰，浪花占了波峰上很大面积，风开始削去波峰上的浪花。',
  'Foam blown off the crests begins to stretch in streaks down the wave faces; crests sometimes take the long shape of storm waves.': '波峰上被风削去的浪花，开始沿着波浪斜面伸长成带状，有时波峰出现风暴波的长波形状。',
  'Streaks of blown foam cover the wave faces and reach into some troughs; the crests are covered in foam.': '风削去的浪花带布满了波浪斜面，并有些地方到达波谷，波峰上布满了浪花层。',
  'Dense foam covers the wave faces and the sea turns white; only parts of the troughs are free of foam.': '稠密的浪花布满了波浪斜面，海面变成白色，只有波谷内某些地方没有浪花。',
  'The whole sea is covered in dense foam, the air is full of spray and visibility is much reduced.': '整个海面布满了稠密的浪花层，空气中充满了水滴和飞沫，能见度显著降低。',
  // ---- data recording
  'Data recording': '数据采集', 'Choose storage folder…': '选择存储文件夹…', 'Session name': '会话名称',
  'Capture rate': '采集频率', 'RGB resolution': 'RGB 分辨率', 'RGB format': 'RGB 格式', 'IR resolution': '红外分辨率',
  'JPEG (quality 92)': 'JPEG（质量 92）', 'PNG (lossless)': 'PNG（无损）', '640 × 512 (native)': '640 × 512（原生）',
  'Save RGB images': '保存 RGB 图像', 'Save IR images': '保存红外图像', 'Save IR temperatures (16-bit)': '保存红外温度（16 位）',
  'Save LiDAR point clouds': '保存激光雷达点云', 'Save labels (YOLO boxes)': '保存标注（YOLO 框）', 'LiDAR coordinates': '激光雷达坐标系',
  'Sensor (x fwd, y left, z up)': '传感器（x 前，y 左，z 上）', 'World (x east, y north, z up)': '世界（x 东，y 北，z 上）',
  'Never drop frames (slows sim)': '不丢帧（必要时减慢仿真）', 'Stop after (s, 0 = manual)': '自动停止（s，0 = 手动）',
  '● Start recording': '● 开始采集', '■ Stop recording': '■ 停止采集', 'Storage location': '存储位置',
  'No folder chosen: the recording downloads as a .zip': '未选择文件夹：采集结果将以 .zip 下载',
  'This browser saves recordings as a .zip download': '此浏览器以 .zip 下载方式保存采集结果',
  'This browser cannot write to a folder: recordings are saved as a .zip download.': '此浏览器不能直接写入文件夹：采集结果将以 .zip 下载保存。',
  'Permission to write to the folder was refused.': '写入文件夹的权限被拒绝。', 'Could not start recording': '无法开始采集',
  'Writing failed': '写入失败', 'Memory limit reached (1.5 GB): choose a folder to record longer.': '已达内存上限（1.5 GB）：请选择文件夹以采集更长时间。',
  // ---- flight route planner and mission flight
  'Flight route': '航线规划', 'Mapping mission': '测绘航线',
  'Click on the sea to add boundary points (3 or more). Drag a point to move it, drag + to add one, Delete removes the selected point. Drag the map to pan, wheel to zoom.':
    '在海面上点击添加边界点（至少 3 个）。拖动点可移动，拖动"+"可新增点，Delete 删除选中的点。拖动地图平移，滚轮缩放。',
  'Mapping area': '测区面积', 'Est. duration': '预计时长', 'Est. photos': '预计照片数', 'Route length': '航线长度', 'Line spacing': '航线间距',
  'Flight lines': '航线条数', 'Clear area': '清除区域', 'Delete selected point': '删除选中点', 'Rectangle area': '矩形区域', 'Center view': '居中显示',
  'Start': '起点', 'End': '终点', 'Altitude': '飞行高度', 'Speed': '飞行速度', 'Course angle': '航线角度',
  'Side overlap (line density)': '旁向重叠率（航线密度）', 'Front overlap': '航向重叠率', 'Margin': '边距', 'Gimbal pitch': '云台俯仰角',
  'Upon completion': '完成动作', 'Return to home': '自动返航', 'Hover': '悬停', 'Repeat route': '重复航线',
  'Record the three sensors during the flight': '飞行时采集三种传感器数据',
  'Uses the Data recording settings (folder, rate, formats).': '使用"数据采集"中的设置（文件夹、频率、格式）。',
  'Change start point': '切换起点', 'Start flight': '开始飞行',
  'ALT': '高度', 'SPD': '速度', 'DIST': '距离', 'ETA': '剩余', 'GIMBAL': '云台',
  'Drag up / down to tilt the gimbal · Space pause · Esc exit': '上下拖动调整云台俯仰 · 空格暂停 · Esc 退出',
  'Press Esc to leave the flight view': '按 Esc 退出飞行视角', 'MAPPING': '测绘中', 'PAUSED': '已暂停', 'COMPLETE': '任务完成',
  'Mission complete: back at the start point, hovering': '任务完成：已返回起点并悬停',
  'Mission complete: hovering at the last waypoint': '任务完成：在最后一个航点悬停',
  // ---- simple / professional interface
  'Quick controls': '快捷设置', 'Simple': '简洁', 'Professional': '专业', 'Interface': '界面',

  // ---- control panel: folders
  'Ocean · MOB controls': '海面 · 落水仿真控制面板',
  'Sky & light': '天空与光照', 'Wind & waves': '风与海浪', 'Water': '海水', 'People overboard': '落水人员',
  'Simulation & camera': '仿真与相机', 'Drone EO / IR cameras': '无人机可见光 / 红外相机', 'Cinematic': '电影效果',
  'Reflections & shadows': '反射与阴影', 'Volumetric clouds': '体积云', 'God rays': '体积光',
  'Weather': '天气', 'Underwater & SAR gear': '水下与搜救装备', 'Interactive water & spray': '交互水面与浪花',
  'Performance': '性能', 'Camera': '相机', 'Exposure & anti-aliasing': '曝光与抗锯齿', 'Bloom & lens': '泛光与镜头',
  'Depth of field & motion blur': '景深与运动模糊', 'Colour grade': '调色',
  // sky & light
  'Time of day (h)': '时间（时）', 'Haze': '霾', 'Cloud cover': '云量', 'High cloud veil': '高云薄幕',
  'Exposure': '曝光', 'Bloom': '泛光', 'Aerial haze ×': '大气透视 ×',
  // wind & waves
  'Wind speed (m/s)': '风速（m/s）', 'Wind from (°)': '风向（来向，°）', 'Fetch (km)': '风区长度（km）',
  'Wave height ×': '波高 ×', 'Steepness (choppy disp.)': '陡度（水平位移）', 'Choppiness (spread)': '波向分散度',
  'Swell height (m)': '涌浪高度（m）', 'Swell from (°)': '涌浪来向（°）', 'Swell wavelength (m)': '涌浪波长（m）',
  'Whitecaps ×': '白浪 ×',
  // water
  'Water colour': '水色', 'Scatter colour': '散射色', 'Turbidity': '浑浊度',
  // people
  'Number in water': '落水人数', 'Camera follows group': '相机跟随人群', 'Show buoyancy probes': '显示浮力采样点',
  'Respawn / regroup': '重新生成 / 聚拢',
  // simulation & camera
  'Time scale': '时间倍率', 'Pause': '暂停', 'Auto-orbit': '自动环绕', 'Reset camera': '重置相机',
  // drone EO / IR
  'Show camera windows': '显示相机窗口', 'Camera view': '相机视角',
  'Follow main view (mouse)': '跟随主视角（鼠标）', 'Drone gimbal': '无人机云台',
  'Drone altitude (m)': '无人机高度（m）', 'Horizontal range (m)': '水平距离（m）',
  'Zoom: vertical FOV (°)': '变焦：垂直视场角（°）', 'Orbit (°/s)': '环绕速度（°/s）',
  'IR palette': '红外伪彩', 'White-hot': '白热', 'Black-hot': '黑热', 'Ironbow': '铁红', 'Rainbow': '彩虹',
  'IR sensor noise ×': '红外传感器噪声 ×', 'Sea temperature (°C)': '海水温度（°C）', 'Air temperature (°C)': '气温（°C）',
  'Relative humidity': '相对湿度',
  // cinematic: camera
  'Camera preset': '相机预设', 'Free orbit': '自由环绕', 'Sea level (bobbing on the swell)': '海平面（随涌浪起伏）',
  'Drone orbit': '无人机环绕', 'Helicopter overhead': '直升机俯视', 'Rescue boat POV': '救援艇视角',
  'Focal length (mm, FF)': '焦距（mm，全画幅）', 'Transition (s)': '过渡时间（s）', 'Handheld shake': '手持抖动',
  'Shake amount': '抖动幅度',
  // cinematic: exposure & anti-aliasing
  'Exposure & anti-aliasing>Exposure': '曝光模式', 'Manual': '手动', 'Auto (eye adaptation)': '自动（人眼适应）', 'Auto: adaptation strength': '自动：适应强度',
  'Auto: min EV': '自动：最小 EV', 'Auto: max EV': '自动：最大 EV', 'Adapt to bright (1/s)': '亮适应速度（1/s）',
  'Adapt to dark (1/s)': '暗适应速度（1/s）', 'Anti-aliasing': '抗锯齿', 'MSAA x4 only': '仅 MSAA x4',
  'TAA + MSAA x4': 'TAA + MSAA x4', 'Sharpen': '锐化',
  // cinematic: bloom & lens
  'Bloom (amount: Sky & light)': '泛光（强度在“天空与光照”）', 'Sun flare (ghosts, starburst)': '太阳眩光（鬼影、星芒）',
  'Ghost intensity': '鬼影强度', 'Starburst': '星芒', 'Lens dirt': '镜头污渍', 'Dirt intensity': '污渍强度',
  'Chromatic aberration': '色差', 'Aberration amount': '色差强度', 'Film grain': '胶片颗粒', 'Grain amount': '颗粒强度',
  'Vignette': '暗角', 'Lens water droplets': '镜头水滴', 'Rain on lens': '镜头雨滴',
  // cinematic: depth of field & motion blur
  'Depth of field': '景深', 'Aperture (f-number)': '光圈（f 值）', 'Focus': '对焦',
  'Auto: subject near centre': '自动：画面中心目标', 'Orbit target': '环绕目标点', 'Manual distance': '手动距离',
  'Manual focus (m)': '手动对焦距离（m）', 'Camera motion blur': '相机运动模糊', 'Shutter angle (°)': '快门角（°）',
  // cinematic: colour grade
  'White balance (K)': '白平衡（K）', 'Tint (green–magenta)': '色调（绿–品红）', 'Contrast': '对比度',
  'Saturation': '饱和度', 'Lift': '暗部（Lift）', 'Gamma': '中间调（Gamma）', 'Gain': '亮部（Gain）',
  // reflections & shadows
  'Enabled': '启用', 'People reflections': '人员倒影', 'Sun shadow maps': '太阳阴影贴图',
  'Body shadows on water': '人体在水面的阴影', 'Contact darkening': '接触暗化', 'IR body reflections': '红外人体反射',
  'Wet skin film': '湿润皮肤水膜',
  // LiDAR
  'Show LiDAR window': '显示激光雷达窗口', 'Sensor': '传感器',
  'Solid-state rosette 70° (240k pts/s)': '固态花瓣扫描 70°（24 万点/秒）',
  'Solid-state long range 15° (240k pts/s)': '固态远距 15°（24 万点/秒）',
  'Spinning 32-beam 360°': '机械旋转 32 线 360°', 'Spinning 64-beam 360°': '机械旋转 64 线 360°',
  'Spinning 128-beam long range': '机械旋转 128 线远距',
  'DJI L2-type, non-repetitive 70°×75° (up to 5 returns)': 'DJI L2 型，非重复扫描 70°×75°（最多 5 回波）',
  'DJI L2-type, repetitive 70°×3° (up to 5 returns)': 'DJI L2 型，重复扫描 70°×3°（最多 5 回波）',
  'Returns per pulse': '每脉冲回波数', '1 (single)': '1（单回波）', '2 (dual)': '2（双回波）', '3 (triple)': '3（三回波）',
  '5 (penta)': '5（五回波）', 'Return number': '回波序号', 'True colour (RGB camera)': '真彩色（RGB 相机）',
  'Integration (s, solid-state)': '积分时间（s，固态型）',
  'Wavelength': '波长', '905 nm (silicon APD)': '905 nm（硅 APD）', '1550 nm (InGaAs)': '1550 nm（铟镓砷）',
  'View': '视图', '3D chase': '3D 跟随', 'Sensor POV (as cameras)': '传感器视角（同相机）', 'Bird\'s-eye': '鸟瞰',
  'Colour': '着色', 'Reflectivity': '反射率', 'Range': '距离', 'Height': '高度', 'Return type (ground truth)': '回波类型（真值）',
  'View yaw (°)': '视图方位角（°）', 'View elevation (°)': '视图俯仰角（°）', 'View distance ×': '视图距离 ×',
  'Point size (px)': '点大小（px）', 'Integration (s, rosette)': '积分时间（s，花瓣扫描）',
  'Spin rate (360° models)': '转速（360° 型）', '360°: level spin axis': '360°：旋转轴保持水平',
  '360°: extra down-tilt (°)': '360°：附加下倾角（°）', 'Range @10% refl. (m)': '10% 反射率测程（m）',
  'Max range (m)': '最大量程（m）', 'Range noise ×': '测距噪声 ×', 'Fog / rain clutter': '雾 / 雨杂波',
  'Retroreflector blooming': '反光材料光晕', 'Export point cloud': '导出点云', 'Export format': '导出格式',
  'PCD (text)': 'PCD（文本）', 'PCD (binary)': 'PCD（二进制）', 'LAS 1.2': 'LAS 1.2', 'LAZ (LASzip)': 'LAZ（LASzip 压缩）', 'LiDAR format': '激光雷达格式',
  // volumetric clouds, god rays
  'Volumetric clouds (off = flat layers)': '体积云（关闭 = 平面云层）', 'Coverage': '覆盖率',
  'Type: stratus → cumulus': '类型：层云 → 积云', 'Cloud base (km)': '云底高度（km）', 'Thickness (km)': '云层厚度（km）',
  'Density ×': '密度 ×', 'Edge erosion ×': '边缘侵蚀 ×', 'Drift / evolution ×': '漂移 / 演变 ×', 'Rain shafts': '雨幡',
  'Crepuscular rays': '云隙光', 'Strength': '强度',
  // weather
  'Preset': '预设', 'Clear': '晴', 'Fair cumulus': '晴间淡积云', 'Overcast': '阴天', 'Storm': '暴风雨',
  'Marine fog': '海雾', 'opt:Preset>Rain': '雨天', 'opt:Weather>Rain': '雨天', 'Weather>Rain': '降雨强度', 'Lightning rate': '闪电频率',
  'Sea fog visibility (km, 0 off)': '海雾能见度（km，0 = 关）', 'Trigger lightning': '触发闪电',
  // underwater & SAR gear
  'Lifebuoy': '救生圈', 'Life raft': '救生筏', 'Debris': '漂浮残骸', 'Rescue vessel (horizon)': '救援船（远处）',
  'Sea-marker dye': '海面染色标记', 'Release new dye': '投放新染色剂', 'Regroup gear near people': '将装备聚拢到人员附近',
  'Camera can dive': '相机可潜入水下', 'Float at surface (split view)': '漂浮在水面（分屏）',
  'Underwater clarity ×': '水下清澈度 ×', 'Underwater & SAR gear>God rays': '水下光束', 'Caustics on bodies': '人体焦散',
  'Marine snow': '海洋雪（悬浮颗粒）', 'Bubbles': '气泡', 'View from below': '从水下仰视', 'Half over / half under': '半水上 / 半水下',
  // interactive water & spray
  'Ripples & wakes': '涟漪与尾迹', 'Ripple strength ×': '涟漪强度 ×', 'Drift wake ×': '漂流尾迹 ×',
  'Churn micro-foam ×': '搅动微泡沫 ×', 'Spray & splashes': '浪花与飞溅', 'Spray amount ×': '浪花量 ×',
  'Whitecap spray / spindrift ×': '白浪飞沫 / 浪尖飘沫 ×', 'Splashes (strokes, slaps) ×': '飞溅（划水、拍击）×',
  // performance
  'Quality': '画质', 'Low': '低', 'Medium': '中', 'High': '高', 'Ultra': '极高',
  'Auto resolution (hold 55+ fps)': '自动分辨率（保持 55+ fps）', 'Render scale': '渲染比例',
};
