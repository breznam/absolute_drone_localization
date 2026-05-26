from svl.tms import FlightZoneDownloader, FlightZone, TileDownloader

# define the flight zone
flight_zone = FlightZone(
    top_left_lat=49.2295258,
    top_left_long=16.5707169,
    bottom_right_lat=49.2273258,
    bottom_right_long=16.5750408,
)

# define the tile downloader
tms_url = "https://mt.google.com/vt/lyrs=s&x={x}&y={y}&z={z}&scale=4"
tile_downloader = TileDownloader(
    url=tms_url,
    channels=3,
    api_key=None,
    headers=None,
    img_format="png",
)

# define the flight zone downloader
flight_zone_downloader = FlightZoneDownloader(
    tile_downloader=tile_downloader,
    flight_zone=flight_zone,
)

# download the tiles and save them as a mosaic
output_path = "data/map3"
flight_zone_downloader.download_tiles_and_save_as_mosaic(
    zoom_level=18,
    output_path=output_path,
    mosaic_format="tiff",
)