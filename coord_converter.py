import math

class CoordConverter:
    """坐标系转换工具（WGS-84 ↔ GCJ-02）"""
    
    # 椭球参数
    a = 6378245.0  # 长半轴
    ee = 0.00669342162296594323  # 偏心率平方
    
    @staticmethod
    def _transform_lat(lon, lat):
        """纬度转换"""
        ret = -100.0 + 2.0 * lon + 3.0 * lat + 0.2 * lat * lat + \
              0.1 * lon * lat + 0.2 * math.sqrt(abs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 *
                math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * math.pi) + 40.0 *
                math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 *
                math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _transform_lon(lon, lat):
        """经度转换"""
        ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + \
              0.1 * lon * lat + 0.1 * math.sqrt(abs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 *
                math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 *
                math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lon / 12.0 * math.pi) + 300.0 *
                math.sin(lon / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _out_of_china(lon, lat):
        """判断是否在中国境外"""
        if lon < 72.004 or lon > 137.8347:
            return True
        if lat < 0.8293 or lat > 55.8271:
            return True
        return False
    
    @classmethod
    def wgs84_to_gcj02(cls, lon, lat):
        """WGS-84 转 GCJ-02"""
        if cls._out_of_china(lon, lat):
            return lon, lat
        
        dlat = cls._transform_lat(lon - 105.0, lat - 35.0)
        dlon = cls._transform_lon(lon - 105.0, lat - 35.0)
        
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlon = (dlon * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        
        mg_lat = lat + dlat
        mg_lon = lon + dlon
        return mg_lon, mg_lat
    
    @classmethod
    def gcj02_to_wgs84(cls, lat, lon):
        """GCJ-02 转 WGS-84"""
        if cls._out_of_china(lon, lat):
            return lat, lon
        
        lon, lat = cls.wgs84_to_gcj02(lon, lat)
        # 近似转换
        return lat, lon
    
    @classmethod
    def convert_coords(cls, coords, from_type, to_type="WGS-84"):
        """批量转换坐标
        Args:
            coords: 坐标列表 [[lat, lon], ...]
            from_type: 源坐标系 ("WGS-84" 或 "GCJ-02")
            to_type: 目标坐标系
        Returns:
            转换后的坐标列表
        """
        if from_type == to_type:
            return coords
        
        result = []
        for lat, lon in coords:
            if from_type == "GCJ-02" and to_type == "WGS-84":
                new_lat, new_lon = cls.gcj02_to_wgs84(lat, lon)
            elif from_type == "WGS-84" and to_type == "GCJ-02":
                new_lon, new_lat = cls.wgs84_to_gcj02(lon, lat)
            else:
                new_lat, new_lon = lat, lon
            result.append([new_lat, new_lon])
        return result

# 南京工业大学附近坐标示例（GCJ-02）
NANJING_TECH_COORDS = {
    "校门口": [32.2322, 118.749],
    "图书馆": [32.2343, 118.751],
    "教学楼": [32.2335, 118.753],
    "体育馆": [32.2315, 118.752],
    "宿舍区": [32.2350, 118.748]
}
