"""
嵌入式地图 HTML 模板模块
========================
功能：提供一个完整的 HTML 页面模板，内嵌 Leaflet.js 地图库，
      通过 Qt WebEngine (QWebEngineView) 在桌面应用中渲染交互式地图。

技术说明：
  - Leaflet 是一个轻量级开源 JavaScript 地图库
  - 地图瓦片来自 CartoDB（基于 OpenStreetMap 数据）
  - 通过 runJavaScript() 方法，Python 端可以调用此页面中的 JS 函数
  - Python 端调用示例：self.web_view.page().runJavaScript("updateLocation(31.23, 121.47, true);")
"""

# 完整的 HTML 页面模板，作为 Python 字符串常量导出
# 该模板被 main_window.py 中的 QWebEngineView 加载显示
MAP_HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>实时地图</title>

    <!-- 加载 Leaflet 1.9.4 版本的 CSS 和 JS（从 CDN 获取） -->
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

    <style>
        /* 让地图容器充满整个页面 */
        body, html, #map {
            margin: 0;
            padding: 0;
            width: 100%;
            height: 100%;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }

        /* 右下角坐标信息浮层样式 */
        .info {
            position: absolute;
            bottom: 20px;
            right: 20px;
            background: rgba(0,0,0,0.7);    /* 半透明黑色背景 */
            color: white;
            padding: 8px 15px;
            border-radius: 8px;              /* 圆角 */
            font-size: 12px;
            z-index: 1000;                   /* 确保浮层在地图上方 */
            pointer-events: none;            /* 鼠标事件穿透，不影响地图操作 */
        }
    </style>
</head>
<body>
    <!-- 地图容器，Leaflet 会将地图渲染到这个 div 中 -->
    <div id="map"></div>

    <!-- 右下角信息浮层：显示当前坐标 -->
    <div class="info" id="coordInfo">等待位置数据...</div>

    <script>
        // ======================== 地图初始化 ========================

        // 创建 Leaflet 地图实例，绑定到 id="map" 的 div 元素
        // setView 参数：[纬度, 经度], 缩放级别
        // 初始中心点为上海 (31.23, 121.47)，缩放级别 13（街道级别）
        var map = L.map('map').setView([31.23, 121.47], 13);

        // 添加地图瓦片图层（Tile Layer）
        // CartoDB light_all 是一种浅色风格的地图，适合数据可视化
        // {s}: 子域名（a, b, c, d），用于并行加载瓦片
        // {z}: 缩放级别
        // {x}, {y}: 瓦片坐标
        // {r}: retina 支持（高清屏显示高清瓦片）
        L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png', {
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a> &copy; CartoDB',
            subdomains: 'abcd',    // 4 个子域名，提升瓦片加载速度
            maxZoom: 19,           // 最大缩放级别（街道级别）
            minZoom: 3             // 最小缩放级别（大洲级别）
        }).addTo(map);

        // ======================== 全局状态变量 ========================

        // currentMarker: 当前骑手位置的地图标记（Marker）
        var currentMarker = null;

        // trailPoints: 轨迹点数组，存储历史位置 [[lat1, lng1], [lat2, lng2], ...]
        var trailPoints = [];

        // trailLine: 轨迹折线（Polyline），用红色线条连接所有历史位置点
        // 样式：橙色(#FF5722)，线宽3px，80%不透明度
        var trailLine = L.polyline([], { color: '#FF5722', weight: 3, opacity: 0.8 }).addTo(map);

        // ======================== Python 可调用的 JS 函数 ========================
        // 以下函数由 Python 端通过 runJavaScript() 调用

        /**
         * 更新骑手在地图上的位置
         *
         * 这是最核心的地图更新函数，Python 端每次收到新数据都会调用它。
         *
         * @param {number} lat - 纬度（degrees）
         * @param {number} lng - 经度（degrees）
         * @param {boolean} addToTrail - 是否将当前位置加入轨迹线（默认 true）
         *                               false 时只移动标记，不画轨迹（用于 setMarkerOnly）
         */
        function updateLocation(lat, lng, addToTrail = true) {
            // 更新右下角坐标显示
            document.getElementById('coordInfo').innerHTML = `经度: ${lng.toFixed(6)}°<br>纬度: ${lat.toFixed(6)}°`;

            // 首次调用：创建地图标记
            if (currentMarker === null) {
                currentMarker = L.marker([lat, lng], {
                    // 使用自定义 div 图标：一个带阴影的橙色圆点
                    icon: L.divIcon({
                        html: '<div style="background-color:#FF5722; width:12px; height:12px; border-radius:50%; border:2px solid white; box-shadow:0 0 4px rgba(0,0,0,0.5);"></div>',
                        iconSize: [16, 16],              // 图标尺寸
                        className: 'custom-div-icon'      // CSS 类名
                    })
                }).addTo(map);
                // 鼠标悬停时显示 "当前位置" 提示
                currentMarker.bindTooltip("当前位置", { permanent: false, direction: 'top' });
            } else {
                // 非首次调用：移动已有标记到新位置（比删除重建更流畅）
                currentMarker.setLatLng([lat, lng]);
            }

            // 如果需要记录轨迹
            if (addToTrail) {
                // 将新位置添加到轨迹点数组
                trailPoints.push([lat, lng]);
                // 限制轨迹点最多 500 个，防止内存无限增长
                // 超过 500 个时移除最早的点（FIFO 队列）
                if (trailPoints.length > 500) trailPoints.shift();
                // 更新折线的控制点，轨迹线会自动重绘
                trailLine.setLatLngs(trailPoints);
            }

            // 平滑移动地图中心，让最新位置始终可见
            // animate: true 启用平滑过渡动画，duration: 0.5 秒
            map.panTo([lat, lng], { animate: true, duration: 0.5 });
        }

        /**
         * 清除地图上的轨迹线
         *
         * 操作：清空轨迹点数组 + 重置折线的控制点
         * 注意：不会删除当前位置标记（currentMarker），只清除历史路径
         */
        function clearTrail() {
            trailPoints = [];          // 清空轨迹数组
            trailLine.setLatLngs([]);  // 重置折线（传入空数组）
        }

        /**
         * 重置地图视角
         *
         * 将地图中心移到指定位置，并放大到缩放级别 15（街区级别）。
         *
         * @param {number} lat - 目标纬度
         * @param {number} lng - 目标经度
         */
        function resetView(lat, lng) {
            if (lat && lng) {
                // 如果提供了坐标，以该坐标为中心
                map.setView([lat, lng], 15);
            } else if (currentMarker) {
                // 否则以当前标记位置为中心
                map.setView(currentMarker.getLatLng(), 15);
            }
        }

        /**
         * 仅更新标记位置，不添加轨迹点
         *
         * 与 updateLocation(lat, lng, false) 功能类似，但逻辑上更独立。
         * 用于需要移动标记但不想画轨迹的场景。
         *
         * @param {number} lat - 纬度
         * @param {number} lng - 经度
         */
        function setMarkerOnly(lat, lng) {
            if (currentMarker === null) {
                // 如果标记还不存在，创建一个（但不添加轨迹）
                updateLocation(lat, lng, false);
            } else {
                // 移动已有标记
                currentMarker.setLatLng([lat, lng]);
            }
            // 更新坐标显示
            document.getElementById('coordInfo').innerHTML = `经度: ${lng.toFixed(6)}°<br>纬度: ${lat.toFixed(6)}°`;
            // 平滑移动地图
            map.panTo([lat, lng], { animate: true, duration: 0.5 });
        }
    </script>
</body>
</html>
"""
