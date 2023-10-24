#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <climits>
#include <algorithm>

using namespace std;

struct Edge
{
  int to, year, time, cost;
};

struct Graph
{
  int n;
  vector<vector<Edge>> adj;

  Graph(int n) : n(n), adj(n) {}

  void addEdge(int from, int to, int year, int time, int cost)
  {
    adj[from].push_back({to, year, time, cost});
    adj[to].push_back({from, year, time, cost});
  }
};

struct Node
{
  int village, year, time, cost;

  bool operator>(const Node &other) const
  {
    if (time == other.time)
    {
      return cost > other.cost;
    }

    return time > other.time;
  }
};

vector<int> dijkstra(const Graph &graph, int source)
{
  vector<int> distance(graph.n, INT_MAX);
  vector<bool> visited(graph.n, false);
  priority_queue<Node, vector<Node>, greater<Node>> pq;

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
      int new_time = node.time + edge.time;
      int new_cost = node.cost + edge.cost;

      if (new_time < distance[edge.to] || (new_time == distance[edge.to] && new_cost < distance[edge.to]))
      {
        distance[edge.to] = new_time;
        pq.push({edge.to, node.year, new_time, new_cost});
      }
    }
  }

  return distance;
}

int prim(const Graph &graph)
{
  int total_cost = 0;
  priority_queue<Node, vector<Node>, greater<Node>> pq;
  vector<bool> in_tree(graph.n, false);

  pq.push({0, 0, 0, 0});

  while (!pq.empty())
  {
    Node node = pq.top();
    pq.pop();

    int village = node.village;

    if (in_tree[village])
    {
      continue;
    }

    in_tree[village] = true;
    total_cost += node.cost;

    for (const Edge &edge : graph.adj[village])
    {
      if (!in_tree[edge.to])
      {
        pq.push({edge.to, edge.year, edge.time, edge.cost});
      }
    }
  }

  return total_cost;
}

int main()
{
  int n, m;
  cin >> n >> m;

  Graph graph(n);

  for (int i = 0; i < m; i++)
  {
    int u, v, a, l, c;
    cin >> u >> v >> a >> l >> c;
    graph.addEdge(u - 1, v - 1, a, l, c);
  }

  vector<int> distances = dijkstra(graph, 0);

  for (int i = 0; i < n; i++)
  {
    cout << distances[i] << endl;
  }

  int first_year_achievable_distances = 0;

  int first_year_arrive_any = 0;

  int lower_cost = prim(graph);

  cout << first_year_achievable_distances << endl;

  cout << first_year_arrive_any << endl;

  cout << lower_cost << endl;

  return 0;
}