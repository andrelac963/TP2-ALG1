#include <iostream>
#include <vector>
#include <queue>
#include <climits>

struct Edge
{
  long int to, year, time, cost;
};

struct Graph
{
  long int n;
  std::vector<std::vector<Edge>> adj;

  Graph(long int n) : n(n), adj(n) {}

  void addEdge(long int from, long int to, long int year, long int time, long int cost)
  {
    adj[from].push_back({to, year, time, cost});
    adj[to].push_back({from, year, time, cost});
  }
};

struct Node
{
  long int village, year, time, cost;

  bool operator>(const Node &other) const
  {
    if (time == other.time)
    {
      return cost > other.cost;
    }

    return time > other.time;
  }
};

std::vector<long int> dijkstra(const Graph &graph, long int source)
{
  std::vector<long int> distance(graph.n, INT_MAX);
  std::vector<bool> visited(graph.n, false);
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  distance[source] = 0;
  pq.push({source, 0, 0, 0});

  while (!pq.empty())
  {
    Node node = pq.top();
    pq.pop();

    if (visited[node.village])
    {
      continue;
    }

    visited[node.village] = true;

    for (const Edge &edge : graph.adj[node.village])
    {
      long int new_time = node.time + edge.time;
      long int new_cost = node.cost + edge.cost;

      if (new_time < distance[edge.to] || (new_time == distance[edge.to] && new_cost < distance[edge.to]))
      {
        distance[edge.to] = new_time;
        pq.push({edge.to, node.year, new_time, new_cost});
      }
    }
  }

  return distance;
}

void primMinimumPathandCost(const Graph &graph, long int &minimum_path_stability_time, long int &spanning_tree_cost)
{
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
  std::vector<bool> in_tree(graph.n, false);

  pq.push({0, 0, 0, 0});

  while (!pq.empty())
  {
    Node node = pq.top();
    pq.pop();

    long int village = node.village;

    if (in_tree[village])
    {
      continue;
    }

    in_tree[village] = true;
    spanning_tree_cost += node.cost;

    if (node.year > minimum_path_stability_time)
    {
      minimum_path_stability_time = node.year;
    }

    for (const Edge &edge : graph.adj[village])
    {
      if (!in_tree[edge.to])
      {
        pq.push({edge.to, edge.year, edge.time, edge.cost});
      }
    }
  }
}

void bfsFirstConnection(const Graph &graph, long int &first_connection_time)
{
  std::vector<bool> visited(graph.n, false);
  std::queue<int> q;

  q.push(0);
  visited[0] = true;

  while (!q.empty())
  {
    long int village = q.front();
    q.pop();

    for (const Edge &edge : graph.adj[village])
    {
      if (!visited[edge.to])
      {
        visited[edge.to] = true;

        if (edge.year > first_connection_time)
        {
          first_connection_time = edge.year;
        }

        q.push(edge.to);
      }
    }
  }
}

int main()
{
  long int n, m;
  std::scanf("%ld %ld", &n, &m);

  Graph graph(n);

  for (int i = 0; i < m; i++)
  {
    long int u, v, a, l, c;
    std::scanf("%ld %ld %ld %ld %ld", &u, &v, &a, &l, &c);
    graph.addEdge(u - 1, v - 1, a, l, c);
  }

  std::vector<long int> distances = dijkstra(graph, 0);

  for (int i = 0; i < n; i++)
  {
    std::cout << distances[i] << std::endl;
  }

  long int minimum_path_stability_time = 0;
  long int first_connection_time = 0;
  long int spanning_tree_cost = 0;

  primMinimumPathandCost(graph, minimum_path_stability_time, spanning_tree_cost);
  bfsFirstConnection(graph, first_connection_time);

  std::cout << minimum_path_stability_time << std::endl;
  std::cout << first_connection_time << std::endl;
  std::cout << spanning_tree_cost << std::endl;

  return 0;
}