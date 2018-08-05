from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import googlemaps

gmaps = googlemaps.Client(key='******API_Key******')  # Replace with the Google Distance Matrix API Key...

class DataProblem():
    """Stores the data for the problem"""
    def __init__(self, location, num_vehicles):
        """Initializes the data for the problem"""
        self._num_vehicles = num_vehicles

        self._locations = [(loc[0], loc[1]) for loc in location]
        self._depot = 0

    @property
    def num_vehicles(self):
        """Gets number of vehicles"""
        return self._num_vehicles

    @property
    def locations(self):
        """Gets locations"""
        return self._locations

    @property
    def num_locations(self):
        """Gets number of locations"""
        return len(self.locations)

    @property
    def depot(self):
        """Gets depot location index"""
        return self._depot


def google_distanceNduration(pos1, pos2):
    dist = gmaps.distance_matrix(pos1, pos2)
    return dist['rows'][0]['elements'][0]['distance']['value'], dist['rows'][0]['elements'][0]['duration']['value']

class CreateDistanceEvaluator(object): 
    """Creates callback to return distance between points."""
    def __init__(self, data):
        """Initializes the distance matrix."""
        self._distances = {}

        # Computing distance between location to have distance callback in O(1)
        for from_node in xrange(data.num_locations):
            self._distances[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._distances[from_node][to_node] = 0
                else:
                    self._distances[from_node][to_node],_ = google_distanceNduration(
                            data.locations[from_node],
                            data.locations[to_node])

    def distance_evaluator(self, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return self._distances[from_node][to_node]

def add_distance_dimension(routing, distance_evaluator, max_vehicle_distance):
    """Add Global Span constraint"""
    distance = "Distance"
    routing.AddDimension(
        distance_evaluator,
        0, # null slack
        max_vehicle_distance, # maximum distance per vehicle
        True, # start cumul to zero
        distance)
    distance_dimension = routing.GetDimensionOrDie(distance)
    distance_dimension.SetGlobalSpanCostCoefficient(100)


class ConsolePrinter():
    """Print solution to console"""
    def __init__(self, data, routing, assignment):
        """Initializes the printer"""
        self._data = data
        self._routing = routing
        self._assignment = assignment

    @property
    def data(self):
        """Gets problem data"""
        return self._data

    @property
    def routing(self):
        """Gets routing model"""
        return self._routing

    @property
    def assignment(self):
        """Gets routing model"""
        return self._assignment

    def print(self):
        """Prints assignment on console"""
        # Inspect solution.
        total_dist = 0
        total_time = 0
        for vehicle_id in xrange(self.data.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            route_time = 0
            while not self.routing.IsEnd(index):
                node_index = self.routing.IndexToNode(index)
                next_node_index = self.routing.IndexToNode(
                    self.assignment.Value(self.routing.NextVar(index)))
                dist, time = google_distanceNduration(
                    self.data.locations[node_index],
                    self.data.locations[next_node_index])
                route_dist += dist
                route_time += time
                plan_output += ' {0} -> '.format(node_index)
                index = self.assignment.Value(self.routing.NextVar(index))

            node_index = self.routing.IndexToNode(index)
            total_dist += route_dist
            total_time += route_time
            plan_output += ' {0}\n'.format(node_index)
            plan_output += 'Distance of the route: {0}m\n'.format(route_dist)
            print(plan_output)
        print('Total Distance of all routes: {0}m'.format(total_dist))
        print('Total Time of all routes: {0}min'.format(total_time/60))


def main():
    """Entry point of the program"""
    # Input locations.
    inp = eval(input())  # Input from node.js
    location = inp['location']
    num_vehicles = inp['num_vehicles']
    max_vehicle_distance = inp['max_vehicle_distance']

    # Instantiate the data problem.    
    data = DataProblem(location, num_vehicles)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    
    # Define weight of each edge
    distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
    add_distance_dimension(routing, distance_evaluator, max_vehicle_distance)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    printer = ConsolePrinter(data, routing, assignment)
    printer.print()


if __name__ == '__main__':
    main()
