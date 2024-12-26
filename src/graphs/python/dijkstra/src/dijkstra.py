import graph as G
import heapq

def dijkstra(graph, start):
    # Priority queue to store (distance, vertex)
    pq = [(0, start)]
    # Dictionary to store the shortest path to each vertex
    distances = {vertex: float('infinity') for vertex in graph.vertices()}
    distances[start] = 0

    while pq:
        current_distance, current_vertex = heapq.heappop(pq)

        # Nodes can only be added once to the priority queue
        if current_distance > distances[current_vertex]:
            continue

        for edge in graph.edges_from_vertex(current_vertex):
            neighbor = edge.destination
            distance = current_distance + edge.distance

            # Only consider this new path if it's better
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))

    return distances

# Example usage:
vA = G.Vertex('A')
vB = G.Vertex('B')
vC = G.Vertex('C')
vD = G.Vertex('D')

graph = G.Graph()
graph.add_vertex(vA)
graph.add_vertex(vB)
graph.add_vertex(vC)
graph.add_vertex(vD)

graph.add_edge(Edge(vA, vB, 1))
graph.add_edge(Edge(vA, vC, 4))
graph.add_edge(Edge(vB, vC, 2))
graph.add_edge(Edge(vB, vD, 5))
graph.add_edge(Edge(vC, vD, 1))

start_vertex = vA
print(dijkstra(graph, start_vertex))