import gmplot, json

gmap = gmplot.GoogleMapPlotter(
	42.29335375,
	-71.26358725,
	19)

with open('../waypoints.json', 'r') as wptfile:
	wpts = json.load(wptfile);

longitudes = [float(wpts[wpt]['longitude']) for wpt in wpts]
latitudes = [float(wpts[wpt]['latitude']) for wpt in wpts]

gmap.scatter(latitudes, longitudes, '#000000', marker=True);
gmap.draw("waypointmap.html")