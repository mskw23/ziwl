from xml.dom import minidom
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from datetime import datetime, timedelta

class TravelData():
    def __init__(self, location_idx, kms, time):
        self.location_idx = location_idx
        self.kms = kms
        self.time = time

class Location():
    def __init__(self, idx, id, is_DC, preferred_fleet, orders, travel_data):
        self.idx = idx
        self.id = id
        self.is_DC = is_DC
        self.preferred_fleet = preferred_fleet
        self.orders = orders
        self.travel_data = travel_data

class Order():
    def __init__(self, idx, id, location_from, location_to,
                 delivery_start, delivery_end, priority,
                 min_temp, max_temp, pallet_type_idx,
                 pallet_quantity, weight_per_pallet):
        self.idx = idx
        self.id = id
        self.location_from = location_from
        self.location_to = location_to
        self.delivery_start = delivery_start
        self.delivery_end = delivery_end
        self.priority = priority
        self.min_temp = min_temp
        self.max_temp = max_temp
        self.pallet_type_idx = pallet_type_idx
        self.pallet_quantity = pallet_quantity
        self.weight_per_pallet = weight_per_pallet

class Pallet():
    def __init__(self, idx, type, length, width, height, ratio_to_stdPallet):
        self.idx = idx
        self.type = type
        self.length = length
        self.width = width
        self.height = height
        self.ratio_to_stdPallet = ratio_to_stdPallet

class Section():
    def __init__(self, min_temp, max_temp, capacity_in_stdPallets):
        self.min_temp = min_temp
        self.max_temp = max_temp
        self.capacity_in_stdPallets = capacity_in_stdPallets


class Fleet():
    def __init__(self, idx, id, cost_per_km, max_weight, max_kms, loading_from_side, preferred_locations, section):
        self.idx = idx
        self.id = id
        self.cost_per_km = cost_per_km
        self.max_weight = max_weight
        self.max_kms = max_kms
        self.loading_from_side = loading_from_side
        self.preferred_locations = preferred_locations
        self.section = section



def get_data(xml):
    xmldoc = minidom.parse(xml)
    xmllocations = xmldoc.getElementsByTagName('Location')
    xmlorders = xmldoc.getElementsByTagName('Order')
    xmlpallets = xmldoc.getElementsByTagName('Pallet')
    xmlfleets = xmldoc.getElementsByTagName('Fleet')

    locations = []
    for xmllocation in xmllocations:
        idx = xmllocation.getAttribute('idx')
        id = xmllocation.getAttribute('id')
        is_DC = xmllocation.getAttribute('is_DC')
        preferred_fleet = xmllocation.getAttribute('preferred_fleet').split(' ')
        locorders = xmllocation.getAttribute('orders').split(' ')
        xmltraveldata = xmllocation.getElementsByTagName('Travel_Info')
        traveldata = []
        for xmltravelinfo in xmltraveldata:
            location_idx = xmltravelinfo.getAttribute('location_idx')
            kms = xmltravelinfo.getAttribute('kms')
            time = xmltravelinfo.getAttribute('time')
            traveldata.append(TravelData(location_idx, kms, time))

        locations.append(Location(idx, id, is_DC, preferred_fleet, locorders, traveldata))

    orders = []
    for xmlorder in xmlorders:
        idx = xmlorder.getAttribute('idx')
        id = xmlorder.getAttribute('id')
        location_from = xmlorder.getAttribute('location_from')
        location_to = xmlorder.getAttribute('location_to')
        delivery_start = xmlorder.getAttribute('delivery_start')
        delivery_end = xmlorder.getAttribute('delivery_end')
        priority = xmlorder.getAttribute('priority')
        min_temp = xmlorder.getAttribute('min_temp')
        max_temp = xmlorder.getAttribute('max_temp')
        pallet_type_idx = xmlorder.getAttribute('pallet_type_idx')
        pallet_quantity = xmlorder.getAttribute('pallet_quantity')
        weight_per_pallet = xmlorder.getAttribute('weight_per_pallet')
        orders.append(Order(idx, id, location_from, location_to,
                            delivery_start, delivery_end, priority,
                            min_temp, max_temp, pallet_type_idx,
                            pallet_quantity, weight_per_pallet))

    pallets = []
    for xmlpallet in xmlpallets :
        idx = xmlpallet.getAttribute('idx')
        type = xmlpallet.getAttribute('type')
        length = xmlpallet.getAttribute('length')
        width = xmlpallet.getAttribute('width')
        height = xmlpallet.getAttribute('height')
        ratio_to_stdPallet = xmlpallet.getAttribute('ratio_to_stdPallet')
        pallets.append(Pallet(idx, type, length, width, height, ratio_to_stdPallet))

    fleets = []
    for xmlfleet in xmlfleets:
        idx = xmlfleet.getAttribute('idx')
        id = xmlfleet.getAttribute('id')
        cost_per_km = xmlfleet.getAttribute('cost_per_km')
        max_weight = xmlfleet.getAttribute('max_weight')
        max_kms = xmlfleet.getAttribute('max_kms')
        loading_from_side = xmlfleet.getAttribute('loading_from_side')
        preferred_locations = xmlfleet.getAttribute('preferred_locations').split(' ')

        min_temp = xmlfleet.getElementsByTagName('Section')[0].getAttribute('min_temp')
        max_temp = xmlfleet.getElementsByTagName('Section')[0].getAttribute('max_temp')
        capacity_in_stdPallets = xmlfleet.getElementsByTagName('Section')[0].getAttribute('capacity_in_stdPallets')

        fleets.append(Fleet(idx, id, cost_per_km, max_weight, max_kms, loading_from_side, preferred_locations,
                            Section(min_temp, max_temp, capacity_in_stdPallets)))


    return [locations, orders, pallets, fleets]

def get_demands(locations, orders):

    demands = []

    for _ in locations:
        demands.append(0)

    for order in orders:
        for id, location in enumerate(locations):
            if order.location_to == location.id:
                demands[id] += 1

    return demands

def get_DC_index(locations):
    for id, location in enumerate(locations):
        if location.is_DC == 'true':
            return id

def route_output_string(routing, plan):
    dropped = []
    for order in range(routing.Size()):
        if (plan.Value(routing.NextVar(order)) == order):
            dropped.append(str(order))

    capacity_dimension = routing.GetDimensionOrDie("Capacity")
    time_dimension = routing.GetDimensionOrDie("Time")
    plan_output = ''

    for route_number in range(routing.vehicles()):
        order = routing.Start(route_number)
        plan_output += 'Route {0}:'.format(route_number)
        if routing.IsEnd(plan.Value(routing.NextVar(order))):
            plan_output += ' Empty \n'
        else:
            while True:
                load_var = capacity_dimension.CumulVar(order)
                time_var = time_dimension.CumulVar(order)
                plan_output += \
                    " {order} Load({load}) Time({tmin}, {tmax}) -> ".format(
                        order=order,
                        load=plan.Value(load_var),
                        tmin=str(timedelta(seconds=plan.Min(time_var))),
                        tmax=str(timedelta(seconds=plan.Max(time_var))))

                if routing.IsEnd(order):
                    plan_output += ' EndRoute {0}. \n'.format(route_number)
                    break
                order = plan.Value(routing.NextVar(order))
        plan_output += "\n"

    return (plan_output, dropped)

def build_vehicle_route(routing, plan, locations, veh_number):

    veh_used = routing.IsVehicleUsed(plan, veh_number)
    print('Vehicle {0} is used {1}'.format(veh_number, veh_used))
    if veh_used:
        route = []
        node = routing.Start(veh_number)
        route.append(locations[routing.IndexToNode(node)])
        while not routing.IsEnd(node):
            route.append(locations[routing.IndexToNode(node)])
            node = plan.Value(routing.NextVar(node))

        route.append(locations[routing.IndexToNode(node)])
        return route
    else:
        return None

def get_start_index(fleets):
    return [0 for _ in fleets]

def main():
    [locations, orders, pallets, fleets] = get_data('ziwl_data.xml')
    print get_demands(locations, orders)

    model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()
    routing = pywrapcp.RoutingModel(len(locations), len(fleets), get_start_index(fleets), get_start_index(fleets), model_parameters)
    parameters = routing.DefaultSearchParameters()

    # Setting first solution heuristic (cheapest addition).
    parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Disabling Large Neighborhood Search, (this is the default behaviour)
    parameters.local_search_operators.use_path_lns = False
    parameters.local_search_operators.use_inactive_lns = False
    # Routing: forbids use of TSPOpt neighborhood,
    parameters.local_search_operators.use_tsp_opt = False

    parameters.time_limit_ms = 10 * 1000  # 10 seconds
    parameters.use_light_propagation = False
    # parameters.log_search = True

    # Set the cost function (distance callback) for each arc, homogenious for
    # all vehicles.
    for fleet in fleets:
        routing.SetFixedCostOfVehicle(int(float(fleet.cost_per_km)), int(fleet.idx))

if __name__ == '__main__':
    main()