import os
import networkx as nx
import osmnx as ox
import numpy as np
import geopandas as gpd
from shapely.ops import linemerge

# save root path to ensure easily navigating back
root_path = os.getcwd()
try:
    # attempt to change folder to TDCDP working folder
    os.chdir(os.getcwd() + '/TDCDP')
    try:
        os.chdir(os.getcwd() + '/rotterdam')
    except:
        raise Exception('rotterdam folder not found')
except:
    raise Exception('TDCDP folder not found')

# Helper function, skip
def kwikqdrdist(lata, lona, latb, lonb):
    """Gives quick and dirty qdr[deg] and dist [m]
       from lat/lon. (note: does not work well close to poles)"""
    re      = 6371000.  # radius earth [m]
    dlat    = np.radians(latb - lata)
    dlon    = np.radians(((lonb - lona)+180)%360-180)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle  = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist    = re * dangle

    qdr     = np.degrees(np.arctan2(dlon * cavelat, dlat)) % 360
    return qdr, dist

def spawndrone(trkid, u, v, r):
    """Function that spawns a drone at a given location.
    trkid = truck id
    u = start node number (spawn location)
    v = customer node number
    r = retrieval node number
    """
    spawn_coords = nodes.get_coordinates().loc[u] # Get initial location location
    target_coords = nodes.get_coordinates().loc[v] # Get target customer location
    retrieval_coords = nodes.get_coordinates().loc[r] # Get target retrieval location
    achdg, _ = kwikqdrdist(spawn_coords.y, spawn_coords.x, target_coords.y, target_coords.x) # get original heading
    acid = 'SP1'
    actype = 'M600'
    acalt = 8 # ft, truck height
    acspd = 0 # kts, start from standstill
    cruise_spd = 25 # kts
    cruise_alt = 100 # ft
    # When the truck arrives at the target node, the drone is released (spawned). Initially, the speed is 0.
    scen_text = f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 CRE {acid} {actype} {spawn_coords.y}' + \
           f' {spawn_coords.x} {achdg} {acalt} {acspd}\n'
    
    # View waypoints of spawned AC for testing. Helps for tracking the route. Remove later.
    scen_text += f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 POS {acid}\n'

    # The spawned drone first has to climb to a safe altitude (avoidance of collisions). Let spawned drone climb.
    scen_text += f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 ALT {acid} 100\n' 
    # Now, the cruise speed can be set.
    scen_text += f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 {acid} ATALT 100 SPDAP {acid} 25\n' 

    scen_text += f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 ADDDELIVERYPOINTS {acid}'
    cust_spd = 5 # kts
    for wp in [spawn_coords, target_coords, retrieval_coords]:
        if wp.x == target_coords.x and wp.y == target_coords.y:
            delivery = 'DELIVERY'
        else:
            delivery = 'nondelivery'
        # Add the text for this waypoint.
        scen_text += f',{wp.y},{wp.x},{cruise_alt},{cruise_spd},TURNSPD,{cust_spd}, {delivery}, 5'

    # Add enter space after waypoints
    scen_text += '\n'

    # Configure properly. Enable its vertical and horizontal navigation
    scen_text += f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 LNAV {acid} ON\n'
    scen_text += f'00:00:00>{trkid} ATDIST {spawn_coords.y} {spawn_coords.x} 0.00025 VNAV {acid} ON\n'

    return scen_text

# The graph of Rotterdam is contained within the rotterdam.gpkg file. The graphml one is just an alternative
# format that is sometimes needed. To visualise the graph nicely, you can install QGIS (qgis.org) and open
# rotterdam.qgz . If the window is just blank, you can right click on the edges layer on the left
# and select "zoom to layer".
# The graph is made up of nodes and edges. You can see the node IDs displayed in QGIS. The arrow on each
# edge represents its designated direction. The edges are also colour coded in function of the "stroke_group"
# designation. We basically wanted to group edges into streets such that the directions of these streets
# is consistent. For this, we used the COINS algorithm (part of the momepy python package) to group
# edges into streets. The stroke_groups are these streets, and each colour represents a street.

# You can import a graph using both the .graphml and the .gpkg. However, the gpkg is nicer to work with
# as geopandas likes it more. So this is how you can load a graph:
try:
    nodes = gpd.read_file('rotterdam.gpkg', layer='nodes')
    edges = gpd.read_file('rotterdam.gpkg', layer='edges')
except:
    raise Exception('Rotterdam graph not found')

# When loading a graph from a gpkg, geopandas will try to (incorrectly) guess the index. 
# So we need to set the correct index.
# Each node gets an "osmid", an OpenStreetMap ID. In this graph however, we set our own custom (simpler)
# osmids in ascending order. However, this ID is needed for working with the OSMNX package.
nodes.set_index(['osmid'], inplace=True)
# Every edge is described by a starting node (u) and an end node (v). These also show the directionality
# of the edge. You can travel from u to v but not from v to u. Key is always 0.
edges.set_index(['u', 'v', 'key'], inplace=True)

# You can see what information these two have by looking at their columns
print('Information in the nodes dataframe:')
print(nodes.columns)
print('Information in the edges dataframe:')
print(edges.columns)

# With these set, the graph can be created. This is a directional graph, so when planning a path through it,
# the directionality will be taken into account.
G = ox.graph_from_gdfs(nodes, edges)

# The undirected graph can also be easily obtained
G_undir = ox.get_undirected(G)

# This is how to plan a path in the graph and then plug it into BlueSky. 
# We need a start and an end node. You can get these by using the node 'osmid' value. The QGIS project displays these.
start_node = 644 # Centraal
end_node = 939 # Blaak

# We can get the nodes of the shortest path there easily. The weight needs to be specified otherwise it will simply
# plan for the lowest number of nodes, which isn't necessarily the shortest.
route_nodes = nx.shortest_path(G, start_node, end_node, weight = 'length')

# But now we simply have a list of nodes, so we want to get the actual geometry of the route in lat-lon coordinates.
# We can do that by using the geometry information contained in the edges database. The nodes are in order, so we
# just need to get the information for each edge.
# We can get the required edges that we need to travel on by simply using the nodes, as each edge is described by
# u and v. So we simply basically create a list that looks like this: [[route_nodes[0], route_nodes[1]], [route_nodes[1], route_nodes[2]], ...]
route_edges = zip(route_nodes[:-1], route_nodes[1:])

# Now we can extract the geometry. In the edges dataframe, the geometry is a Shapely linestring. So we use the u and v (and key = 0) to
# find the required edges.
route_geoms = [edges.loc[(u, v, 0), 'geometry'] for u, v in route_edges]

# These are a bunch of lines though, so we need to combine them into one.
route_line = linemerge(route_geoms)

# Now that we have the line, we can get the waypoints in LAT LON. The graph is in LON LAT, but BlueSky works with LAT LON.
route_waypoints = list(zip(route_line.xy[1], route_line.xy[0]))
route_lats = route_line.xy[1]
route_lons = route_line.xy[0]

# Every time a drone makes a turn in BlueSky, if it doesn't change its speed, it will overshoot the turn by a lot, which we don't
# want. So we implemented a type of waypoint called a "turn waypoint". If a waypoint is marked as such, the aircraft will know
# to slow down first and then make the turn. We need to identify the waypoints for which this is needed.
# To do this, we want to check the angles between the path legs.
i = 1 # Start at second waypoint
turns = [True] # Doesn't matter what the first waypoint is designated as, so just have it as true.
for lat_cur, lon_cur in route_waypoints[1:-1]:
    # Get the previous and the next waypoint
    lat_prev, lon_prev = route_waypoints[i-1]
    lat_next, lon_next = route_waypoints[i+1]
    # Get the angle
    a1, _ = kwikqdrdist(lat_prev,lon_prev,lat_cur,lon_cur)
    a2, _ = kwikqdrdist(lat_cur,lon_cur,lat_next,lon_next)
    angle=abs(a2-a1)
    if angle>180:
        angle=360-angle
    # In general, we noticed that we don't need to slow down if the turn is smaller than 25 degrees. However, this will depend
    # on the cruise speed of the drones.
    if angle > 25:
        turns.append(True)
    else:
        turns.append(False)
    i+= 1

# It's generally a good idea to also always consider the last waypoint a turn so the aircraft slows down
turns.append(True)

# Now we have everything we need to create a BlueSky scenario for this path. So first, we'll add some commands to pan to the correct
# location and zoom in.
scen_text = f'00:00:00>PAN {route_lats[0]} {route_lons[0]}\n' # Pan to the origin
scen_text += '00:00:00>ZOOM 50\n\n' # Zoom in

# Let's create the drone.
# We want it to already be facing the correct direction. So let's find out what is the direction of the first leg.
achdg, _ = kwikqdrdist(route_lats[0], route_lons[0], route_lats[1], route_lons[1])

# Now we can initialise the create command. We mostly use the Matrice 600 (M600) model for drone simulations.
# CRE command takes Aircraft ID, Aircraft Type, Lat, Lon, hdg, alt[ft], spd[kts]
acid = 'D1'
actype = 'Truck'
acalt = 0 # ft, ground altitude
acspd = 5 if turns[1] else 25 #kts, set it as 5 if the first waypoint is a turn waypoint.
scen_text += f'00:00:00>CRE {acid} {actype} {route_lats[0]} {route_lons[0]} {achdg} {acalt} {acspd}\n'

# After creating it, we want to add all the waypoints. We can do that using the ADDWAYPOINTS command.
# ADDWAYPOINTS can chain waypoint data in the following way:
# ADDWAYPOINTS ACID LAT LON ALT SPD Turn? TurnSpeed
# SPD here can be set as the cruise speed so the aircraft knows how fast to fly
cruise_spd = 25 #kts
cruise_alt = acalt # Keep it constant throughout the flight
# Turn speed of 5 kts usually works well
turn_spd = 5 #kts

# So let's create this command now
scen_text += f'00:00:00>ADDDELIVERYPOINTS {acid}' # First add the root of the command
# Then start looping through waypoints
for wplat, wplon, turn in zip(route_lats, route_lons, turns):
    # Check if this waypoint is a turn
    if turn:
        wptype = 'TURNSPD'
    else:
        wptype = 'FLYBY'
    # Add the text for this waypoint. It doesn't matter if we always add a turn speed, as BlueSky will
    # ignore it if the wptype is set as FLYBY
    scen_text += f',{wplat},{wplon},{cruise_alt},{cruise_spd},{wptype},{turn_spd}, Nondelivery, 5'

# Add a newline at the end of the addwaypoints command
scen_text += '\n'

# Let drones spawn at calculated target locations

scen_text += spawndrone('D1', 652, 2320, 655)

# Now we just need to make sure that the aircraft is configured properly. So we enable its vertical and horizontal navigation
scen_text += f'00:00:00>LNAV {acid} ON\n'
scen_text += f'00:00:00>VNAV {acid} ON\n'
# Turn trail on, tracing for testing
scen_text += '00:00:00>TRAIL ON \n'
scen_text += f'00:00:00>POS {acid}\n'
# We also want the aircraft removed when it reaches the destination. We can use an ATDIST command for that.
destination_tolerance = 3/1852 # we consider it arrived if it is within 3 metres of the destination, converted to nautical miles.
scen_text += f'00:00:00>{acid} ATDIST {route_lats[-1]} {route_lons[-1]} {destination_tolerance} DEL {acid}\n'

# Change directory to scenario folder
try:
    os.chdir(root_path + '/scenario')
except:
    raise Exception('Scenario folder not found')

# Now we save the text in a scenario file
with open('RTMtest.scn', 'w') as f:
    f.write(scen_text)