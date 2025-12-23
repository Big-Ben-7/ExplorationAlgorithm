#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <getopt.h>
#include <array>
#include <set>
#ifdef _OPENMP
#include <pthread.h>
#include <omp.h>
#endif
using namespace std;

#define DEF_KF  0.15
#define DEF_KL  0.3
#define DEF_MAX_FUEL  3000
#define DEF_MAX_OXYGEN  2000
#define DEF_MAX_FOOD  2000
#define INF  numeric_limits<double>::infinity()
#define EPSILON  std::numeric_limits<double>::epsilon()

// Command line option input
int scanSlope = 51;
int idealCheck = 1;
int nproc = 0;
int maxPaths = 2000;
int pathFactor = 5;
int useBidirection = 1;
int useOldYens = 0;
int skipM1 = 0;
int skipM2 = 0;
int skipM3 = 0;
int skipSingle = 1;
int skipOld = 1;
int testMode = 0;
string fileName = "";
double maxFuel = DEF_MAX_FUEL;
double maxOxygen = DEF_MAX_OXYGEN;
double maxFood = DEF_MAX_FOOD;

enum Resupply
{
  NONE,
  OXYGEN,
  FOOD,
  BOTH
};
struct Boundary
{
  double xb, yb;
  Boundary(double x, double y)
  {
    xb = x;
    yb = y;
  }
};
struct Point
{
  double x, y;
  Point()
  {
    x = y = 0;
  }
  Point(double newX, double newY)
  {
    x = newX, y = newY;
  }
};
struct PlanetData
{
  string name;
  Point position;
  Resupply resupply;
  double hostileRadius;
  PlanetData(string n, double x, double y, Resupply r, double h)
  {
    name = n, position = Point(x, y), resupply = r, hostileRadius = h;
  }
};
/*struct Edge
{
  int node1, node2;
  double distance;
  Edge(int n1, int n2, double d)
  {
    node1 = n1, node2 = n2, distance = d;
  }
  bool operator<(const Edge &other) const
  {
    int other1 = other.node1;
    if (node1 == other1)
      return node2 < other.node2;
    else
      return node1 < other1;
  }
};*/
/*struct GraphNode
{
  int nodeIndex;
  double cost;
  GraphNode(int n, double c)
  {
    nodeIndex = n;
    cost = c;
  }
  bool operator<(const ConnectedNode &other) const
  {
    return nodeIndex < other.nodeIndex;
  }
};*/
struct NodeData
{
  double minCost;
  int previousNode;
  NodeData(double initialCost)
  {
    minCost = initialCost;
    previousNode = -1;
  }
};
/*// Represents one node in a node tree
struct TreeNode
{
  int nodeIndex;
  TreeNode *parent;
  double cost;
  vector<TreeNode *> children;
  TreeNode(int i, TreeNode *p, double c)
  {
    nodeIndex = i;
    parent = p;
    cost = c;
  }
};*/
struct BestSlope
{
  int index;
  double slope;
  double score;
  double v1, v2;
  BestSlope()
  {
    index = -1;
    score = INF;
  }
};
struct Path
{
  vector<int> path;
  double cost;
  Path(vector<int> p, double c)
  {
    path = p;
    cost = c;
  }
  bool operator<(const Path& other) const {
    if (fabs(cost - other.cost) < EPSILON)
      return path < other.path; // Allow different paths with same cost
    return cost < other.cost; // Sort by cost in Yen's algorithm set
  }
};

chrono::steady_clock::time_point startTime;
vector<double> heuristics, backHeuristics;
vector<PlanetData> planets;
int nodeCount, lastNodeIndex;
// 3 adjacency matrix structure used to efficiently represent graph, containing distance, initial cost, and cost between two node indices
vector<vector<double>> nodeCosts;
vector<vector<double>> nodeDistances;
vector<vector<double>> initialCosts; // Used in Yen's algorithm when restoring removed edges/nodes
//vector<vector<ConnectedNode>> adjacencyList; // Adjacency list structure used to efficiently represent graph
//vector<TreeNode *> possiblePaths;

const double kf = DEF_KF;
const double kl = DEF_KL;
// Speed optimization by first derivative test
// Cost = c(x) = time + fuel + food + oxygen = d/v + kf*d*v^2 + kl*d/v + kl*d/v
// c'(x) = -d/v^2 + 2*kf*d*v - kl*d/v^2 - kl*d/v^2 = (-d + 2*kf*d*v^3 - kl*d - kl*d)/v^2
// Cost minimized at c'(x) = 0 => -d + 2*kf*d*v^3 - kl*d - kl*d = 0 => v = cbrt((1 + kl+kl) / (2*kf))
const double idealSpeed = cbrt((1 + 2 * kl) / (2 * kf));
// Ideal speed disregarding consumption of one kl term (one resupply remaining in path)
const double singleResupplySpeed = cbrt((1 + kl) / (2 * kf));
// Ideal speed disregarding consumption of both kl terms (both resupply types remaining)
const double doubleResupplySpeed = 1 / cbrt(2 * kf);
string mapName;

string toLower(string str)
{
  string str2 = str;
  for (auto &c : str2)
    c = static_cast<unsigned char>(tolower(static_cast<unsigned char>(c)));
  return str2;
}
string trim(string str)
{
  string str2 = str;
  int i, j, l = str.length();
  for (i = 0; i < l; i++)
  {
    if (!isspace(str[i]))
      break;
  }
  for (j = l - 1; j >= i; j--)
  {
    if (!isspace(str[j]))
      break;
  }
  str2 = str2.substr(i, j - i + 1);
  return str2;
}
bool isValidPlanet(string planetName)
{
  bool alphanumeric;
  for (auto &c : planetName)
  {
    if (isalnum(c))
    {
      if (!alphanumeric)
        alphanumeric = true;
      continue;
    }
    switch (c)
    {
      // Spaces, dashes, and parentheses allowed
      case ' ':
      case '-':
      case '(':
      case ')':
        continue;
      default:
        return false;
    }
  }
  // Planet names can't be all special characters!
  return alphanumeric;
}
double calculateDistance(const Point pos1, const Point pos2)
{
  double dx = pos2.x - pos1.x, dy = pos2.y - pos1.y;
  return sqrt(dx * dx + dy * dy);
}
bool edgeInHostileRadius(const int startNode, const int endNode)
{
  // Calculates intersection of line through (x1,y1) and (y1,y2) with circle of radius r at (x3,y3)
  Point &start = planets[startNode].position, end = planets[endNode].position;
  // Line eq. (parametric): x = x1 + cos(theta)*d, y = y1 + sin(theta)*d where d = distance along line and theta = atan(slope)
  double x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y, rise = y2 - y1, run = x2 - x1;
  double theta = atan2(rise, run), cosTheta = cos(theta), sinTheta = sin(theta); // Precompute trig for efficiency
  double l = run * run + rise * rise; // Length of line
  for (auto &planet : planets)
  {
    if (planet.hostileRadius == 0)
      continue;
    Point &center = planet.position;
    // Circle eq.: (x-x3)^2 + (y-y3)^2 = r^2
    double x3 = center.x, y3 = center.y, r = planet.hostileRadius, dx = x1 - x3, dy = y1 - y3;
    // Plugging line eqs. into circle eq. and applying trig identities yields quadratic a*d^2 + b*d + c = 0
    double b = 2 * dx * cosTheta + 2 * dy * sinTheta, c = dx * dx + dy * dy - r * r; // a=1
    double discriminant = b * b - 4 * c;
    // If discriminant < 0, no solution and edge does not cross hostile circle
    if (discriminant < 0)
      continue;
    double d1 = (-b + sqrt(discriminant)) / 2, d2 = (-b - sqrt(discriminant)) / 2; // Roots from quadratic formula
    // If both values for d are lower or higher than edge length, hostile circle is still not crossed
    if ((d1 < 0 && d2 < 0) || (d1 * d1 > l && d2 * d2 > l))
      continue;
    return true;
  }
  return false;
}
// Returns cost (time + resource consumption) of edge connecting two nodes
double calculateCost(int node1, int node2)
{
  double d = nodeDistances[node1][node2];
  double fuelConsumed, oxygenConsumed, foodConsumed, t, speed;
  switch (planets[node2].resupply)
  {
    case OXYGEN:
      speed = singleResupplySpeed, t = d / speed;
      oxygenConsumed = 0;
      foodConsumed = kl * t;
      break;
    case FOOD:
      speed = singleResupplySpeed, t = d / speed;
      foodConsumed = 0;
      oxygenConsumed = kl * t;
      break;
    default:
      speed = idealSpeed, t = d / speed;
      oxygenConsumed = foodConsumed = kl * t;
      break;
  }
  fuelConsumed = kf * d * speed * speed;
  return t + fuelConsumed + oxygenConsumed + foodConsumed;
}

// Bisection algorithm to determine roots of a function (used in speed optimization)
double bisection(double a, double b, const double x0, const double y0, double (*func)(const double, const double, const double), const double tolerance = 1e-6)
{
  double c;
  c = 0.5 * (a + b);
  double fa = func(a, x0, y0);
  double fb = func(b, x0, y0);
  double accuracy = tolerance * 1e-3;
  while (fabs(b - a) > tolerance * c)
  {
    double fc = func(c, x0, y0);
    if (fabs(fc) < accuracy)
      return c;
    if ((fa < 0 && fc < 0) || (fa > 0 && fc > 0))
    {
      a = c;
      fa = fc;
    }
    else
    {
      b = c;
      fb = fc;
    }
    c = 0.5 * (a + b);
  }
  return c;
}
double derivFunc(const double theta, const double x0, const double y0)
{
  double cosTheta = cos(theta), sinTheta = sin(theta);
  return x0 * cosTheta - y0 * sinTheta - (cosTheta * cosTheta - sinTheta * sinTheta);
}
double func(const double theta, const double x0, const double y0)
{
  double cosTheta = cos(theta), sinTheta = sin(theta);
  return x0 * sinTheta + y0 * cosTheta - sinTheta * cosTheta;
}

// Extracts data from inputted CSV map with error handling
int parseCSV()
{
  if (fileName.empty())
  {
    cout << endl << "Input map file: ";
    getline(cin, fileName);
  }
  ifstream iFile;
  string line = "";
  mapName = fileName.substr(0, fileName.rfind('.'));
  iFile.open(fileName);
  if (!iFile.is_open())
  {
    cout << "Error: file \"" << fileName << "\" not found. Please make sure that the entered name is correct and that the CSV file is in the same folder as this project." << endl;
    return -1;
  }

  // Start computation time immediately after user enters file name.
  startTime = chrono::steady_clock::now();

  // Header line error handling
  do
  {
    getline(iFile, line);
  } while (trim(line).empty()); // Skip empty lines
  if (count(line.begin(), line.end(), ',') != 5)
  {
    cout << "Error: Entry 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
    return -1;
  }
  int commaIndex;
  string headerSections[] = {"planet name", "x", "y", "type", "resupply", "hostile_radius"};
  for (int i = 0; i < 5; i++)
  {
    commaIndex = line.find(',');
    if (trim(toLower(line.substr(0, commaIndex))) != headerSections[i])
    {
      cout << "Error: Entry 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);
  }

  // Extract data with error handling
  string planetName, type, resupply, substring;
  int lineNumber = 1;
  double x, y, hostileRadius;
  while (getline(iFile, line))
  {
    if (trim(line).empty())
      continue; // Skip empty lines
    lineNumber++;
    int extraCommas = count(line.begin(), line.end(), ',') - 5;
    if (extraCommas > 0)
    {
      if (extraCommas == 1)
        cout << "Error: Entry " << lineNumber << " has an extra comma." << endl;
      else
        cout << "Error: Entry " << lineNumber << " has " << extraCommas << " extra commas." << endl;
      return -1;
    }
    else if (extraCommas < 0)
    {
      extraCommas *= -1;
      if (extraCommas == 1)
        cout << "Error: Entry " << lineNumber << " needs an extra comma." << endl;
      else
        cout << "Error: Entry " << lineNumber << " needs " << extraCommas << " extra commas." << endl;
      return -1;
    }

    // Planet name
    commaIndex = line.find(',');
    planetName = trim(line.substr(0, commaIndex));
    if (planetName.empty() || !isValidPlanet(planetName))
    {
      cout << "Error: Invalid planet name \"" << planetName << "\" found on entry " << lineNumber << ". Planet names must be alphanumeric (spaces, dashes, and parentheses accepted)." << endl;
      return -1;
    }
    int planetCount = planets.size();
    for (int i = 0; i < planetCount; i++)
    {
      if (planets[i].name == planetName)
      {
        cout << "Error: Duplicate planet name \"" << planetName << "\" found on entries " << i + 2 << " and " << lineNumber << "." << endl;
        return -1;
      }
    }
    line = line.substr(commaIndex + 1);

    // X-coordinate
    commaIndex = line.find(',');
    substring = trim(line.substr(0, commaIndex));
    try
    {
      x = stod(substring);
    }
    catch (...)
    {
      cout << "Error: The x-coordinate (\"" << substring << "\") of planet " << planetName << " (entry " << lineNumber << ") must be numerical." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);

    // Y-coordinate
    commaIndex = line.find(',');
    substring = trim(line.substr(0, commaIndex));
    try
    {
      y = stod(substring);
    }
    catch (...)
    {
      cout << "Error: The y-coordinate (\"" << substring << "\") of planet " << planetName << " (entry " << lineNumber << ") must be numerical." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);

    // Type
    commaIndex = line.find(',');
    type = trim(toLower(line.substr(0, commaIndex)));
    if (type != "neutral" && type != "hostile")
    {
      cout << "Error: The type (\"" << type << "\") of planet " << planetName << " (entry " << lineNumber << ") must be \"neutral\" or \"hostile\"." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);

    // Resupply
    commaIndex = line.find(',');
    resupply = trim(toLower(line.substr(0, commaIndex)));
    Resupply resupplyEnum;
    if (resupply == "oxygen")
      resupplyEnum = OXYGEN;
    else if (resupply == "food")
      resupplyEnum = FOOD;
    else if (resupply.empty() || resupply == "none")
      resupplyEnum = NONE;
    else
    {
      cout << "Error: The resupply (\"" << resupply << "\") of planet " << planetName << " (entry " << lineNumber << ") must be \"\" (none), \"food\", or \"oxygen\"." << endl;
      return -1;
    }
    line = trim(line.substr(commaIndex + 1));

    // Hostile radius
    try
    {
      hostileRadius = stod(line);
    }
    catch (...)
    {
      cout << "Error: The hostile radius (\"" << line << "\") of planet " << planetName << " (entry " << lineNumber << ") must be numerical." << endl;
      return -1;
    }
    if (type == "neutral" && hostileRadius != 0)
    {
      cout << "Error: The hostile radius (" << hostileRadius << ") of neutral planet " << planetName << " (entry " << lineNumber << ") must be 0." << endl;
      return -1;
    }
    else if (type == "hostile")
    {
      if (lineNumber == 2)
      {
        cout << "Error: The first planet (" << planetName << ") cannot be hostile." << endl;
        return -1;
      }
      else if (hostileRadius <= 0)
      {
        cout << "Error: The hostile radius (" << hostileRadius << ") of hostile planet " << planetName << " (entry " << lineNumber << ") must be positive." << endl;
        return -1;
      }
    }
    planets.emplace_back(planetName, x, y, resupplyEnum, hostileRadius);
  }
  if (lineNumber < 3)
  {
    cout << "Error: There must be at least 2 planets in the map." << endl;
    return -1;
  }
  if (planets.back().hostileRadius != 0)
  {
    cout << "Error: The last planet (" << planets.back().name << ") cannot be hostile." << endl;
    return -1;
  }
  nodeCount = lineNumber - 1;
  lastNodeIndex = nodeCount - 1;
  iFile.close();
  return 0;
}

// Provides pathfinding setup
void initializeLists()
{
  // Heuristic constant for A* pathfinding (minimum cost per unit distance traveled)
  double heuristic = (1 + kl) / singleResupplySpeed + kf * singleResupplySpeed * singleResupplySpeed;
  heuristics = vector<double>(nodeCount), backHeuristics = vector<double>(nodeCount);
  Point sourcePos = planets[0].position, sinkPos = planets[lastNodeIndex].position;
  nodeCosts = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  nodeDistances = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  initialCosts = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  for (int i = 0; i < nodeCount; i++)
  {
    double hostileRadius = planets[i].hostileRadius;
    // Fills adjacency list with list of nodes connected to each node
    for (int j = i + 1; j < nodeCount; j++)
    {
      // Eliminate edges crossing hostile radius
      if (hostileRadius != 0 || edgeInHostileRadius(i, j))
      {
        //nodeDistances[i][j] = nodeDistances[j][i] = INF;
        nodeCosts[i][j] = initialCosts[i][j] = nodeCosts[j][i] = initialCosts[j][i] = INF;
        if (j == lastNodeIndex)
          heuristics[i] = heuristic * calculateDistance(planets[i].position, sinkPos);
        if (i == 0)
          backHeuristics[j] = heuristic * calculateDistance(planets[j].position, sourcePos);
        continue;
      }
      double distance = nodeDistances[i][j] = nodeDistances[j][i] = calculateDistance(planets[i].position, planets[j].position);
      /*if (i != lastNodeIndex)*/ nodeCosts[i][j] = initialCosts[i][j] = calculateCost(i, j);
      //else nodeCosts[i][j] = initialCosts[i][j] = INF;
      /*if (j != lastNodeIndex)*/ nodeCosts[j][i] = initialCosts[j][i] = calculateCost(j, i);

      // A* heuristic for each node
      if (j == lastNodeIndex)
      {
        //nodeCosts[j][i] = initialCosts[j][i] = INF;
        heuristics[i] = heuristic * distance; // Use Euclidian distance to target as heuristic for forward search
      }
      if (i == 0)
        backHeuristics[j] = heuristic * distance; // Use Euclidian distance to source as heuristic for backward search
    }
    nodeCosts[i][i] = initialCosts[i][i] = INF;
  }
  /*for (int i = 0; i < nodeCount; i++)
  {
    for (int j = 0; j < nodeCount; j++)
    {
      cout << i << " to " << j << ": " << initialCosts[i][j] << endl;
    }
  }*/
}

/*// Implements modified Bellman-Ford algorithm to find approximate best path
void bellmanFord()
{
  int edgeCount = edgeList.size();
  bool unrelaxed = true;
  // Terminates once one iteration has passed with no relaxations
  while (unrelaxed)
  {
    unrelaxed = false;
    for (int i = 0; i < edgeCount; i++)
    {
      Edge &edge = edgeList[i];
      int node1 = edge.node1, node2 = edge.node2;
      NodeData &node1Data = nodes[node1];
      NodeData &node2Data = nodes[node2];
      double newCost = calculateCost(node1, node2) + node1Data.minCost;
      if (newCost < node2Data.minCost)
      {
        // Relax edge
        node2Data.minCost = newCost;
        node2Data.bestPath = node1Data.bestPath;
        node2Data.bestPath.push_back(node2);
        if (!unrelaxed)
          unrelaxed = true;
      }
    }
  }
}*/

/*// Implements modified Dijkstra's algorithm to find approximate best path
void dijkstra()
{
  // Priority queue of <cost, index> to pick most promising (least cost) node
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
  vector<bool> visited = vector<bool>(nodeCount);
  pq.emplace(0, 0);
  while (!pq.empty())
  {
    int node1 = pq.top().second;
    pq.pop();
    if (visited[node1]) continue;
    visited[node1] = true;
    if (node1 == lastNodeIndex) // Reached target
      return;
    
    NodeData &node1Data = nodes[node1];
    for (auto &node2 : adjacencyList[node1]) // Iterate through adjacent nodes
    {
      int node2 = node2.first;
      if (visited[node2]) continue;
      double cost = node2.second, newCost = cost + node1Data.minCost;
      NodeData &node2Data = nodes[node2];
      if (newCost < node2Data.minCost)
      {
        // Relax edge
        node2Data.minCost = newCost;
        node2Data.previousNode = node1;
        pq.emplace(newCost, node2); // Push negative cost so that queue sorts lowest cost (least negative) at top
      }
    }
  }
}*/

// Implements modified A* algorithm to find single best-path from source to sink node
int aStar(vector<int> &bestPath, const int source, const int sink)
{
  vector<NodeData> nodes;
  for (int i = 0; i < nodeCount; i++)
  {
    if (i == source)
      nodes.emplace_back(0); // Set distance to initial node to 0
    else
      nodes.emplace_back(INF); // Set pre-relaxed distance to all other nodes to infinity
  }
  
  // Priority queue of <cost, index> to pick most promising (least cost) node
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
  pq.emplace(0, source);
  vector<bool> visited(nodeCount);
  while (!pq.empty())
  {
    int node1 = pq.top().second;
    pq.pop();
    if (visited[node1])
      continue;
    visited[node1] = true;
    if (node1 == sink) // Reached target CHECK - should I use this in bidirectional
      break;
    
    double minCost = nodes[node1].minCost;
    for (int node2 = 0; node2 < nodeCount; node2++) // Iterate through adjacent nodes
    {
      double cost = nodeCosts[node1][node2];
      if (visited[node2] || cost == INF)
        continue;
      double newCost = cost + minCost;
      NodeData &node2Data = nodes[node2];
      if (newCost < node2Data.minCost)
      {
        // Relax edge
        node2Data.minCost = newCost;
        node2Data.previousNode = node1;
        pq.emplace(newCost + heuristics[node2], node2);
      }
    }
  }
  
  // Reconstruct path found by A* algorithm
  NodeData node = nodes.back();
  bestPath.push_back(sink);
  while (true)
  {
    int previousNode = node.previousNode;
    if (previousNode < 0)
      break;
    bestPath.push_back(previousNode);
    node = nodes[previousNode];
  }
  if (bestPath.size() <= 1)
    return -1;
  reverse(bestPath.begin(), bestPath.end());
  return 0;
}

// Implements modified A* algorithm with bidirectional search to find single best-path from source to sink node
int bidirectionAStar(vector<int> &bestPath, const int source, const int sink)
{
  // Forward and backward priority queues of <cost, index> to pick most promising (least cost) node
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pqF, pqB;
  pqF.emplace(0, source);
  pqB.emplace(0, sink);
  // Forward and backward g-scores (costs), initialized pre-relaxed to infinity
  vector<double> gF(nodeCount, INF), gB(nodeCount, INF);
  gF[source] = gB[sink] = 0; // Set distance to forward and backward search initial nodes (source and sink) to 0
  vector<bool> visitedF(nodeCount, false), visitedB(nodeCount, false); // Visited nodes in forward and backward search
  vector<int> parentsF(nodeCount, -1), parentsB(nodeCount, -1); // Parents of nodes in forward and backward search for path reconstruction

  double bestCost = INF;
  int meetingNode = -1;
  while (!pqF.empty() && !pqB.empty())
  {
    pair<double, int> forwardPair = pqF.top(), backwardPair = pqB.top();
    double costF = forwardPair.first, costB = backwardPair.first;
    if (bestCost < INF)
    {
      // If sum of node costs exceeds best cost, end search
      if (costF + costB >= bestCost)
        break;
    }

    // Forward search
    pqF.pop();
    int node1 = forwardPair.second;
    if (!visitedF[node1])
    {
      visitedF[node1] = true;
      if (visitedB[node1])
      {
        // If node visited in both forward and backward search, check if it's the best meeting node
        double newCost = gF[node1] + gB[node1];
        if (newCost < bestCost)
        {
          bestCost = newCost;
          meetingNode = node1;
        }
      }
      double g1 = gF[node1];
      for (int node2 = 0; node2 < nodeCount; node2++) // Iterate through adjacent nodes
      {
        double cost = nodeCosts[node1][node2];
        if (cost == INF || visitedF[node2]) // CHECK - should i check if node2 is visited?
          continue;
        double newG = g1 + cost;
        if (newG < gF[node2])
        {
          // Relax edge
          gF[node2] = newG;
          parentsF[node2] = node1;
          pqF.emplace(newG + heuristics[node2], node2);
        }
      }
    }

    // Backward search
    pqB.pop();
    node1 = backwardPair.second;
    if (!visitedB[node1])
    {
      visitedB[node1] = true;
      if (visitedF[node1])
      {
        // If node visited in both forward and backward search, check if it's the best meeting node
        double newCost = gF[node1] + gB[node1];
        if (newCost < bestCost)
        {
          bestCost = newCost;
          meetingNode = node1;
        }
      }
      double g1 = gB[node1];
      for (int node2 = 0; node2 < nodeCount; node2++) // Iterate through adjacent nodes
      {
        double cost = nodeCosts[node2][node1]; // CHECK - should i reverse these
        if (cost == INF || visitedB[node2])
          continue;
        double newG = g1 + cost;
        if (newG < gB[node2])
        {
          // Relax edge
          gB[node2] = newG;
          parentsB[node2] = node1;
          pqB.emplace(newG + backHeuristics[node2], node2);
        }
      }
    }
  }

  // Reconstruct path found by A* algorithm
  if (meetingNode == -1)
    return -1;
  vector<int> leftPath, rightPath;
  int node = meetingNode;
  while (node != -1)
  {
    // Build left path before meeting node
    leftPath.push_back(node);
    if (node == source)
      break;
    node = parentsF[node];
  }
  if (leftPath.back() != source)
    return -1; // Path didn't connect to source
  reverse(leftPath.begin(), leftPath.end());
  node = meetingNode;
  while (node != -1 && node != sink)
  {
    // Build right path after meeting node
    node = parentsB[node];
    if (node != -1)
      rightPath.push_back(node);
  }
  if (meetingNode != sink && (rightPath.empty() || rightPath.back() != sink)) // CHECK - meeting node is sink often
    return -1; // Path didn't connect to sink
  /*if (meetingNode == sink)
    cout << "Meeting node is sink" << endl;*/
  bestPath = leftPath;
  bestPath.insert(bestPath.end(), rightPath.begin(), rightPath.end());
  return bestPath.empty() ? -1 : 0;
}

void printPath(const string id, const vector<int> &path, const double score)
{
  if (id.length() > 0)
    cout << id << ": ";
  for (int i : path)
    cout << i << ", ";
  cout << score << endl;
}

// Implements modified Yen's algorithm to find pathCount approximately-best paths
int oldYens(vector<vector<int>> &bestPaths, const int pathCount)
{
  vector<int> bestPath;
  if ((useBidirection == 1)? bidirectionAStar(bestPath, 0, lastNodeIndex) : aStar(bestPath, 0, lastNodeIndex) == -1) return -1;
  bestPaths.push_back(bestPath); // Find best path from source (Earth) to sink (Target)
  //printPath("Best", bestPath, 0);
  vector<vector<int>> potentialPaths;
  for (int k = 1; k < pathCount; k++)
  {
    vector<int> lastBestPath = bestPaths.back();
    int n = lastBestPath.size() - 1;
    for (int i = 0; i < n; i++)
    {
      int spurNode = lastBestPath[i]; // Retrieve spur node from previous shortest path
      vector<int> rootPath(lastBestPath.begin(), lastBestPath.begin() + i + 1); // Sequence of nodes from source to spur node
      for (auto path : bestPaths)
      {
        vector<int> segment(path.begin(), path.begin() + i + 1);
        if (rootPath == segment)
          nodeCosts[path[i]][path[i+1]] = INF; // Remove links that are part of previous shortest paths which share same root path
      }
      for (int j = 0; j < i; j++)
      {
        for (int m = 0; m < nodeCount; m++)
          nodeCosts[lastBestPath[j]][m] = nodeCosts[m][lastBestPath[j]] = INF; // Remove all nodes in root path from graph
      }

      vector<int> spurPath;
      int error = (useBidirection == 1)? bidirectionAStar(spurPath, spurNode, lastNodeIndex) : aStar(spurPath, spurNode, lastNodeIndex); // Calculate spur path from spur node to sink
      // Restore removed edges/nodes
      for (auto path : bestPaths)
      {
        vector<int> segment(path.begin(), path.begin() + i + 1);
        if (rootPath == segment)
          nodeCosts[path[i]][path[i+1]] = initialCosts[path[i]][path[i+1]];
      }
      for (int j = 0; j < i; j++)
      {
        for (int m = 0; m < nodeCount; m++)
        {
          nodeCosts[lastBestPath[j]][m] = initialCosts[lastBestPath[j]][m];
          nodeCosts[m][lastBestPath[j]] = initialCosts[m][lastBestPath[j]];
        }
      }
      
      if (error == -1) 
        continue;
      vector<int> totalPath = rootPath;
      totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end()); // Combine root and spur path into complete path
      if (find(potentialPaths.begin(), potentialPaths.end(), totalPath) == potentialPaths.end())
      {
        //printPath("Potential - " + to_string(k), totalPath, 0);
        potentialPaths.push_back(totalPath); // If complete path not already in potential paths, add it
      }
    }

    if (potentialPaths.empty()) // No spur paths (e.g. all already exhausted or none exist)
      break;
    // Find best path out of potential paths to add to list of pathCount best paths
    n = potentialPaths.size();
    double bestCost = INF;
    int bestPathIndex = -1;
    for (int i = 0; i < n; i++)
    {
      vector<int> &path = potentialPaths[i];
      int pathLength = path.size();
      double cost = 0;
      for (int j = 1; j < pathLength; j++)
      {
        cost += initialCosts[path[j-1]][path[j]];
      }
      if (cost < bestCost)
      {
        bestCost = cost;
        bestPathIndex = i;
      }
    }
    //cout << "Best path index: " << bestPathIndex << endl;
    //cout << "Best cost: " << bestCost << endl;
    if (bestPathIndex >= 0) 
    {
      //printPath("Best - k=" + to_string(k) + ", i=" + to_string(bestPathIndex), potentialPaths[bestPathIndex], bestCost);
      bestPaths.push_back(potentialPaths[bestPathIndex]);
      potentialPaths.erase(potentialPaths.begin() + bestPathIndex); 
    }
  }
  return 0;
}

// Implements modified Yen's algorithm to find pathCount approximately-best paths
int yens(vector<vector<int>> &bestPaths, int pathCount)
{
  vector<int> bestPath, sameRootPaths, rootPath, spurPath, totalPath;
  if ((useBidirection == 1)? bidirectionAStar(bestPath, 0, lastNodeIndex) : aStar(bestPath, 0, lastNodeIndex) == -1) return -1;
  //printPath("Best", bestPath, 0);
  bestPaths.push_back(bestPath); // Find best path from source (Earth) to sink (Target)
  set<Path> potentialPaths; // Use set to avoid duplicates and automatic sorting
  for (int k = 1; k < pathCount; k++)
  {
    vector<int> &lastBestPath = bestPaths.back();
    int n = lastBestPath.size() - 1;
    for (int i = 0; i < n; i++)
    {
      rootPath.assign(lastBestPath.begin(), lastBestPath.begin() + i + 1); // Sequence of nodes from source to spur node
      sameRootPaths.clear(); // Store paths with removed edges to speed up restoring
      int bestPathsCount = bestPaths.size();
      for (int j = 0; j < bestPathsCount; j++)
      {
        vector<int> &path = bestPaths[j];
        if (rootPath.size() <= path.size() && equal(rootPath.begin(), rootPath.end(), path.begin()))
        {
          sameRootPaths.push_back(j);
          nodeCosts[path[i]][path[i+1]] = INF; // Remove edges from spur nodes of other paths sharing same same root to prevent duplicate paths
        }
      }
      for (int j = 0; j < i; j++)
      {
        for (int m = 0; m < nodeCount; m++)
          nodeCosts[lastBestPath[j]][m] = nodeCosts[m][lastBestPath[j]] = INF; // Remove all nodes in root path from graph to prevent cycles
      }
      int spurNode = lastBestPath[i]; // Retrieve spur node from previous shortest path
      spurPath.clear();
      int error = (useBidirection == 1)? bidirectionAStar(spurPath, spurNode, lastNodeIndex) : aStar(spurPath, spurNode, lastNodeIndex); // Calculate spur path from spur node to sink

      // Restore removed edges/nodes
      int sameRootCount = sameRootPaths.size();
      for (int j = 0; j < sameRootCount; j++)
      {
        vector<int> &path = bestPaths[sameRootPaths[j]];
        nodeCosts[path[i]][path[i+1]] = initialCosts[path[i]][path[i+1]];
      }
      for (int j = 0; j < i; j++)
      {
        for (int m = 0; m < nodeCount; m++)
        {
          nodeCosts[lastBestPath[j]][m] = initialCosts[lastBestPath[j]][m];
          nodeCosts[m][lastBestPath[j]] = initialCosts[m][lastBestPath[j]];
        }
      }

      if (error == -1)
        continue;
      totalPath = rootPath;
      totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end()); // Combine root and spur path into complete path
      int pathLength = totalPath.size();
      double cost = 0;
      for (int j = 1; j < pathLength; j++)
        cost += initialCosts[totalPath[j-1]][totalPath[j]];
      //printPath("Potential - " + to_string(k), totalPath, cost);
      potentialPaths.emplace(totalPath, cost); // Adds complete path sorted by ascending cost, avoiding duplicates
    }

    if (potentialPaths.empty()) // No spur paths (e.g. all already exhausted or none exist)
      break;
    auto it = potentialPaths.begin();
    const Path &path = *it; // First element of set returns best path
    //printPath("Best - " + to_string(k), path.path, path.cost);
    bestPaths.push_back(path.path);
    potentialPaths.erase(it);
  }
  return 0;
}

/*// Determines list of best paths by finding paths that have a cost close to approximate best path but 
void findPathTree(TreeNode *head, double minCost, double maxCost)
{
  static int id = 0;
  id++;
  if (head->nodeIndex == 0)
  {
    if (head->cost >= minCost) possiblePaths.push_back(head);
    return;
  }
  vector<TreeNode *> &children = head->children;
  for (auto &node : adjacencyList[head->nodeIndex])
  {
    int index = node.first;
    cout << id << ", " << head->nodeIndex << ", " << index << ", " << maxCost << endl;
    TreeNode *p = head->parent;
    bool skip = false;
    while (p)
    {
      if (index == p->nodeIndex)
      {
        skip = true;
        break;
      }
      p = p->parent;
    }
    if (skip)
      continue;
    double cost = node.second + head->cost;
    cout << id << ", " << head->nodeIndex << ", " << index << ", " << head->cost << ", " << cost << endl;
    if (cost >= maxCost)
      continue;
    children.push_back(new TreeNode(index, head, cost));
    findPathTree(children.back(), minCost, maxCost);
  }
}
*/

// Optimizes speed of individual graph edge in optimizeSpeed
int getIdealSpeed(double &v1, double &v2, double &score, const double m,
                  const double b, const double c, vector<Boundary> &bs)
{
  double outerIntersection = 1 / sqrt(1 + m * m);
  double innerIntersection = 0.0;
  double xi, x1, y1;
  for (auto &boundary : bs)
  {
    x1 = boundary.xb;
    y1 = boundary.yb;
    if (fabs(x1) < EPSILON)
    {
      xi = y1 / m;
    }
    else if (fabs(y1) < EPSILON)
    {
      xi = x1;
    }
    else
    {
      xi = x1 + y1 / m;
    }
    if (xi > innerIntersection)
      innerIntersection = xi;
    // cout << "xi:" << i << "," << m << "," << x1 << "," << y1 << "," << xi << "," << innerIntersection << "," << outerIntersection << endl;
  }
  if (innerIntersection >= outerIntersection)
    return -1;
  double v1Ideal = cbrt(0.5 * b / c);
  double di = 1e-5 * (outerIntersection - innerIntersection);
  double v1Min = innerIntersection + di, v1Max = outerIntersection - di;
  int returnVal;
  if (v1Ideal < v1Min)
  {
    v1 = v1Min;
    returnVal = 0;
  }
  else if (v1Ideal > v1Max)
  {
    v1 = v1Max;
    returnVal = 2;
  }
  else
  {
    v1 = v1Ideal;
    returnVal = 1;
  }
  v2 = m * v1;
  score = b / v1 + c * v1 * v1;

  return returnVal;
}

// Optimizes speed of entire path for Method 3 in optimizePath
int optimizeSpeed(double &v1, double &v2, double &score, int i0, int im, int i1, vector<int> &path, vector<double> &mb, 
                  vector<double> &speeds, vector<double> &fuel, vector<double> &oxygen, vector<double> &food)
{
  int returnVal = -1;
  int i;
  double fuel0 = fuel[i0 - 1], oxygen0 = oxygen[i0 - 1], food0 = food[i0 - 1];
  int ni = i1 - i0 + 2;
  vector<double> cs(ni), ci(ni);
  cs[0] = 0.0;
  ci[0] = 0.0;
  double dx = 0.0, dy = 0.0, sx = 0.0, sy = 0.0;
  double zx = 0.0, zy = 0.0, mx = 0.0, my = 0.0;
  for (i = i0; i <= i1; i++)
  {
    double di = nodeDistances[path[i-1]][path[i]];
    double ds2 = di * speeds[i] * speeds[i];
    double dos = di / speeds[i];
    double dz2 = cbrt(0.5 * mb[i] / kf);
    dz2 = di * dz2 * dz2;
    if (i <= im)
    {
      dx += di;
      sx += ds2;
      mx += dos * mb[i];
      zx += dz2;
    }
    else
    {
      dy += di;
      sy += ds2;
      my += dos * mb[i];
      zy += dz2;
    }
    // cout << "mb:" << i <<","<<di<<","<<dos<<","<<mb[i]<<","<<dos*mb[i]<<endl;
    cs[i - i0 + 1] = cs[i - i0] + ds2;
    ci[i - i0 + 1] = ci[i - i0] + dos;
  }
  vector<Boundary> bs;
  double vx = sqrt(kf / fuel0 * sx);
  double vy = sqrt(kf / fuel0 * sy);
  for (int ir = 0; ir < 2; ir++)
  {
    Resupply rtp;
    double q0;
    if (ir == 0)
    {
      rtp = OXYGEN;
      q0 = oxygen0;
    }
    else
    {
      rtp = FOOD;
      q0 = food0;
    }
    int bestIndex = i0 - 1;
    for (i = i0; i <= i1; i++)
    {
      int node = path[i];
      if (planets[node].resupply == rtp || i == i1)
      {
        double ix = 0.0, iy = 0.0;
        double bx = vx * kl / q0, by = vy * kl / q0;
        if (i <= im)
        {
          ix = ci[i - i0 + 1] - ci[bestIndex - i0 + 1];
          bs.emplace_back(bx * ix, 0.0);
        }
        else if (bestIndex < im)
        {
          ix = ci[im - i0 + 1] - ci[bestIndex - i0 + 1];
          iy = ci[i - i0 + 1] - ci[im - i0 + 1];
          bs.emplace_back(bx * ix, by * iy);
        }
        else
        {
          iy = ci[i - i0 + 1] - ci[bestIndex - i0 + 1];
          bs.emplace_back(0, by * iy);
        }
        // cout << "bs:" << rtp << "," << q0 << "," << ir << "," << ix << "," << iy << "," << bx*ix << "," << by*iy << "," << i << "," << bestIndex << "," << im << "," << i0 << endl;
        bestIndex = i;
        q0 = ir == 0 ? maxOxygen : maxFood;
      }
    }
  }

  double thetaMin = 0.0;
  double thetaMax = M_PI_2;
  for (i = 0; i < bs.size(); i++)
  {
    if (fabs(bs[i].xb) < EPSILON)
    {
      thetaMin = max(thetaMin, asin(bs[i].yb));
    }
    else if (fabs(bs[i].yb) < EPSILON)
    {
      thetaMax = min(thetaMax, acos(bs[i].xb));
    }
    else
    {
      double tm = bisection(0, M_PI_2, bs[i].xb, bs[i].yb, derivFunc);
      thetaMin = max(thetaMin, bisection(0, tm, bs[i].xb, bs[i].yb, func));
      thetaMax = min(thetaMax, bisection(tm, M_PI_2, bs[i].xb, bs[i].yb, func));
    }
    if (thetaMin >= thetaMax)
    {
      return -1;
    }
    // cout << "th:" << i << "," << bs[i].xb << "," << bs[i].yb << "," << thetaMin << "," << thetaMax << endl;
  }
  double idealSlope = sqrt(zy / zx);
  double ds = (thetaMax - thetaMin) * 1e-5;
  thetaMin += ds;
  thetaMax -= ds;
  double minSlope = tan(thetaMin);
  double maxSlope = tan(thetaMax);
  double a, b, c, s;
  // cout << "f0: " << fuel0 << "," << oxygen0 << "," << food0 << "," << dx << "," << dy << endl;
  // cout << "slope: " << i0 << "," << i1 << "," << idealSlope << ", " << minSlope << ", " << maxSlope << endl;
  if (idealSlope < minSlope)
    idealSlope = minSlope;
  else if (idealSlope > maxSlope)
    idealSlope = maxSlope;
  int ib = 0;
  double v1b = 0.0, v2b = 0.0, sb = INF;
  if (scanSlope > 3)
  {
    double dt = (thetaMax - thetaMin) / (scanSlope - 1);
    vector<double> ts(scanSlope), ss(scanSlope);
    BestSlope bestSlope;
    ts[0] = thetaMin;
    ss[0] = INF;
    for (i = 1; i < scanSlope; i++)
    {
      ts[i] = ts[i - 1] + dt;
      ss[i] = INF;
    }
#pragma omp declare reduction(min_slope:BestSlope : omp_out = (omp_in.score < omp_out.score) ? omp_in : omp_out) \
        initializer(omp_priv = BestSlope())
#pragma omp parallel for reduction(min_slope : bestSlope) default(shared) private(i, s, b, c, returnVal)
    for (i = 0; i < scanSlope; i++)
    {
      s = tan(ts[i]);
      b = mx * vx + my * vy / s;
      c = fuel0 + fuel0 * s * s;
      returnVal = getIdealSpeed(v1, v2, score, s, b, c, bs);
      if (returnVal >= 0)
      {
        ss[i] = score;
        if (score < bestSlope.score)
        {
          bestSlope.index = i;
          bestSlope.score = score;
          bestSlope.v1 = v1;
          bestSlope.v2 = v2;
        }
      }
    }
    ib = bestSlope.index;
    v1b = bestSlope.v1;
    v2b = bestSlope.v2;
    sb = bestSlope.score;
    if (ib < 0)
      return -1;
    if (ib > 0 && ib < scanSlope - 1 &&
        ss[ib - 1] < INF && ss[ib + 1] < INF &&
        ss[ib - 1] > ss[ib] && ss[ib + 1] > ss[ib])
    {
      double s01 = (ss[ib] - ss[ib - 1]) / (ts[ib] - ts[ib - 1]);
      double s02 = (ss[ib + 1] - ss[ib - 1]) / (ts[ib + 1] - ts[ib - 1]);
      double t01 = ts[ib] + ts[ib - 1];
      double t02 = ts[ib + 1] + ts[ib - 1];
      a = (s02 - s01) / (t02 - t01);
      b = s01 - a * t01;
      if (a > 1e-10)
      {
        s = -0.5 * b / a;
        if (s > ts[ib - 1] && s < ts[ib + 1])
        {
          s = tan(s);
          b = mx * vx + my * vy / s;
          c = fuel0 + fuel0 * s * s;
          returnVal = getIdealSpeed(v1, v2, score, s, b, c, bs);
          v1b = v1;
          v2b = v2;
          sb = score;
        }
      }
    }
  }
  if (scanSlope <= 3 || idealCheck > 0)
  {
    b = mx * vx + my * vy / idealSlope;
    c = fuel0 + fuel0 * idealSlope * idealSlope;
    returnVal = getIdealSpeed(v1, v2, score, idealSlope, b, c, bs);
    if (idealCheck > 1)
    {
      cout << "sf:" << returnVal << "," << idealSlope << "," << score << "," << sb << "," << v1 << "," << v1b << ", " << v2 << ", " << v2b << ", " << minSlope << ", " << maxSlope << endl;
    }
    if (score > sb)
    {
      v1 = v1b;
      v2 = v2b;
    }
  }
  else
  {
    v1 = v1b;
    v2 = v2b;
  }
  v1 *= speeds[im] / vx;
  v2 *= speeds[im + 1] / vy;
  // cout << "v:" << v1 << "," << v2 << "," << speeds[im] << "," << speeds[im+1] << endl;
  return returnVal;
}

// Optimizes resource usage on path with 3 methods (which increase in both robustness and computation time)
int optimizePath(vector<int> &path, vector<double> &speeds, vector<double> &fuel, vector<double> &oxygen, vector<double> &food)
{
  int error = 0;
  bool foundOxygenResupply = false, foundFoodResupply = false;
  int pathLength = path.size();
  vector<double> mb(pathLength);
  
  // Method 1: set ideal speed by disregard both, one, or none of the kl terms if node is before both, one or none of the resupplies
  for (int i = pathLength - 1; i >= 1; i--)
  {
    // Iterate backward to find last oxygen and food resupplies
    Resupply resupply = planets[path[i]].resupply;
    switch (resupply)
    {
      case OXYGEN:
        if (!foundOxygenResupply)
        {
          foundOxygenResupply = true;
          if (!foundFoodResupply)
          {
            speeds[i] = singleResupplySpeed;
            mb[i] = 1 + kl;
          }
          else
          {
            speeds[i] = doubleResupplySpeed;
            mb[i] = 1.0;
          }
        }
        else
        {
          if (!foundFoodResupply)
          {
            speeds[i] = singleResupplySpeed;
            mb[i] = 1 + kl;
          }
          else
          {
            speeds[i] = doubleResupplySpeed;
            mb[i] = 1.0;
          }
        }
        break;
      case FOOD:
        if (!foundFoodResupply)
        {
          foundFoodResupply = true;
          if (!foundOxygenResupply)
          {
            speeds[i] = singleResupplySpeed;
            mb[i] = 1.0 + kl;
          }
          else
          {
            speeds[i] = doubleResupplySpeed;
            mb[i] = 1.0;
          }
        }
        else
        {
          foundFoodResupply = true;
          if (!foundOxygenResupply)
          {
            speeds[i] = singleResupplySpeed;
            mb[i] = 1.0 + kl;
          }
          else
          {
            speeds[i] = doubleResupplySpeed;
            mb[i] = 1.0;
          }
        }
        break;
      default:
        if (foundOxygenResupply && foundFoodResupply)
        {
          speeds[i] = doubleResupplySpeed;
          mb[i] = 1.0;
        }
        else if (foundOxygenResupply && !foundFoodResupply)
        {
          speeds[i] = singleResupplySpeed;
          mb[i] = 1 + kl;
        }
        else if (foundFoodResupply && !foundOxygenResupply)
        {
          speeds[i] = singleResupplySpeed;
          mb[i] = 1 + kl;
        }
        else
        {
          speeds[i] = idealSpeed;
          mb[i] = 1 + 2 * kl;
        }
        break;
    }
  }
  double fuelLeft = maxFuel, oxygenLeft = maxOxygen, foodLeft = maxFood;
  // It can be mathematically proven (eg. with Lagrange multipliers) that the ideal speed is the same for all nodes with the same resupplyAfter type
  for (int i = 1; i < pathLength; i++)
  {
    int node = path[i];
    double speed = speeds[i], distance = nodeDistances[path[i-1]][node], t = distance / speed, consumed = kl * t;
    foodLeft -= consumed, oxygenLeft -= consumed;
    if (error == 0 && (oxygenLeft < 0.0001 || foodLeft < 0.0001))
      error = -1;
    fuelLeft -= kf * distance * speed * speed;
    fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
    switch (planets[node].resupply)
    {
      case OXYGEN:
        oxygenLeft = maxOxygen;
        break;
      case FOOD:
        foodLeft = maxFood;
        break;
      default:
        break;
    }
  }
  if (skipM1 == 0 && error == 0 && fuelLeft >= 0.0001)
    return 0;

  // Method 2: If Method 1 (set ideal case) fails, adjust all speeds by constant factor
  if (skipM2 == 0)
  {
    double totalFuelConsumed = maxFuel - fuelLeft;
    double speedFactor = sqrt((maxFuel - 0.0001) / (totalFuelConsumed));
    fuelLeft = maxFuel, oxygenLeft = maxOxygen, foodLeft = maxFood;
    error = 0;
    if (totalFuelConsumed >= maxFuel)
    {
      for (int i = 1; i < pathLength; i++)
      {
        int node = path[i];
        speeds[i] *= speedFactor;
        double speed = speeds[i], distance = nodeDistances[path[i-1]][node], t = distance / speed, consumed = kl * t;
        fuelLeft -= kf * distance * speed * speed, foodLeft -= consumed, oxygenLeft -= consumed;
        if (oxygenLeft < 0.0001 || foodLeft < 0.0001)
        {
          error = -1;
          break;
        }
        fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
        switch (planets[node].resupply)
        {
          case OXYGEN:
            oxygenLeft = maxOxygen;
            break;
          case FOOD:
            foodLeft = maxFood;
            break;
          default:
            break;
        }
      }
    }
    else
    {
      for (int i = 1; i < pathLength; i++)
      {
        int node = path[i];
        double distance = nodeDistances[path[i-1]][node], speed = speeds[i];
        double minOxygenSpeed = distance / ((oxygenLeft - 0.0001) / kl);
        double minFoodSpeed = distance / ((foodLeft - 0.0001) / kl);
        double minSpeed = max(minOxygenSpeed, minFoodSpeed), maxSpeed = speedFactor * speed;
        if (minSpeed > maxSpeed)
        {
          error = -1;
          break;
        }
        if (speed < minSpeed)
        {
          speeds[i] = speed = minSpeed;
          error = -2;
        }
        double consumed = kl * distance / speed;
        fuelLeft -= kf * distance * speed * speed, oxygenLeft -= consumed, foodLeft -= consumed;
        fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
        switch (planets[node].resupply)
        {
          case OXYGEN:
            oxygenLeft = maxOxygen;
            break;
          case FOOD:
            foodLeft = maxFood;
            break;
          default:
            break;
        }
      }
    }
    if (error == 0)
      return 0;
  }
  
  // Method 3: If Method 2 (constant factor) fails, optimize each node's speeds using a combination of algorithmic, analytical, and graphical means
  if (skipM3 == 0)
  {
    fuelLeft = maxFuel, oxygenLeft = maxOxygen, foodLeft = maxFood;
    double speed, speedNext, score;
    for (int i = 1; i < pathLength; i++)
    {
      int node = path[i];
      double distance = nodeDistances[path[i-1]][node];
      if (pathLength == 1)
        speed = speeds[i];
      else
      {
        if (i < pathLength - 1)
        {
          if (optimizeSpeed(speed, speedNext, score, i, i, pathLength - 1, path, mb, speeds, fuel, oxygen, food) == -1)
            return -1;
          speeds[i] = speed;
        }
        else
          speed = speeds[i] = speedNext;
      }

      double t = distance / speed, consumed = kl * t;
      fuelLeft -= kf * distance * speed * speed, foodLeft -= consumed, oxygenLeft -= consumed;
      if (fuelLeft <= 0 || oxygenLeft <= 0 || foodLeft <= 0)
      {
        error = -1;
        break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      switch (planets[node].resupply)
      {
        case OXYGEN:
          oxygenLeft = maxOxygen;
          break;
        case FOOD:
          foodLeft = maxFood;
          break;
        default:
          break;
      }
    }
  }
  
  if (fuelLeft > 0 && oxygenLeft > 0 && foodLeft > 0)
    return 0;
  else
    return -1;
}
/*
int oldOptimizePath(vector<int> &path, vector<double> &speeds, vector<double> &fuel, vector<double> &oxygen, vector<double> &food)
{
  int pathLength = path.size();
  int lastFoodResupply = -1, lastOxygenResupply = -1;
  for (int i = pathLength - 1; i >= 0; i--)
  {
    switch (planets[path[i]].resupply)
    {
      case OXYGEN:
        if (lastOxygenResupply == -1)
          lastOxygenResupply = i;
        break;
      case FOOD:
        if (lastFoodResupply == -1)
          lastFoodResupply = i;
        break;
      default:
        break;
    }
    if (lastOxygenResupply != -1 && lastFoodResupply != -1)
      break;
  }
  int resupply1 = min(lastFoodResupply, lastOxygenResupply);
  int resupply2 = max(lastFoodResupply, lastOxygenResupply);
  double totalFuelConsumed = 0;
  int node0 = 0;
  for (int i = 0; i <= resupply1; i++)
  {
    // Before last oxygen and food refill, optimize speed based on fuel
    speeds[i] = doubleResupplySpeed;
    int node = path[i];
    totalFuelConsumed += kf * nodeDistances[node0][node] * doubleResupplySpeed * doubleResupplySpeed;
    node0 = node;
  }
  for (int i = resupply1 + 1; i <= resupply2; i++)
  {
    // Before last oxygen/food refill, optimize speed based on fuel and food/oxygen
    speeds[i] = singleResupplySpeed;
    int node = path[i];
    totalFuelConsumed += kf * nodeDistances[node0][node] * singleResupplySpeed * singleResupplySpeed;
    node0 = node;
  }
  for (int i = resupply2 + 1; i < pathLength; i++)
  {
    // After refills, optimize speed based on fuel, food, and oxygen
    speeds[i] = idealSpeed;
    int node = path[i];
    totalFuelConsumed += kf * nodeDistances[node0][node] * idealSpeed * idealSpeed;
    node0 = node;
  }
  // If fuel drops to 0, reduce speed by constant factor
  double speedFactor = sqrt((maxFuel - 1e-10) / totalFuelConsumed);
  double fuelLeft = maxFuel, oxygenLeft = maxOxygen, foodLeft = maxFood;
  node0 = 0;
  if (totalFuelConsumed >= maxFuel)
  {
    for (int i = 0; i < pathLength; i++)
    {
      int node = path[i];
      speeds[i] *= speedFactor;
      double speed = speeds[i], distance = nodeDistances[node0][node], t = distance / speed;
      double consumed = kl * t;
      fuelLeft -= kf * distance * speed * speed, oxygenLeft -= consumed, foodLeft -= consumed;
      // If oxygen or food then drop to 0, path is invalid because food/oxygen and fuel have opposite dependencies on speed
      if (oxygenLeft < 0.0001 || foodLeft < 0.0001)
        return -1;
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      switch (planets[node].resupply)
      {
        case OXYGEN:
          oxygenLeft = maxOxygen;
          break;
        case FOOD:
          foodLeft = maxFood;
          break;
        default:
          break;
      }
      node0 = node;
    }
  }
  else
  {
    for (int i = 0; i < pathLength; i++)
    {
      int node = path[i];
      double distance = nodeDistances[node0][node], speed = speeds[i];
      double minOxygenSpeed = distance / ((oxygenLeft - 1e-10) / kl);
      double minFoodSpeed = distance / ((foodLeft - 1e-10) / kl);
      double minSpeed = max(minOxygenSpeed, minFoodSpeed), maxSpeed = speedFactor * speed;
      if (minSpeed > maxSpeed)
        return -1;
      if (speed < minSpeed)
        speeds[i] = speed = minSpeed;
      double consumed = kl * distance / speed, fuelConsumed = kf * distance * speed * speed;
      fuelLeft -= fuelConsumed;
      switch (planets[node].resupply)
      {
        case OXYGEN:
          oxygenLeft = maxOxygen, foodLeft -= consumed;
          break;
        case FOOD:
          foodLeft = maxFood, oxygenLeft -= consumed;
          break;
        default:
          oxygenLeft -= consumed, foodLeft -= consumed;
          break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      node0 = node;
    }
  }
  if (fuelLeft > 0 && oxygenLeft > 0 && foodLeft > 0)
    return 0;
  else
    return -1;
  return 0;
}
*/
void printData(vector<int> &path, vector<double> &speeds, vector<double> &fuel, vector<double> &oxygen, vector<double> &food)
{
  string startPlanet = planets[0].name;
  cout << endl << endl << "PROGRAM OUTPUT" << endl;
  cout << "Starting Planet: " << startPlanet << ", Target Planet: " << planets[lastNodeIndex].name << endl;
  cout << "Name of Map Used: " << mapName << endl;
  cout << "Planet Travel Order (excluding " << startPlanet << "): " << endl << endl;
  cout << right << setw(14) << "Planet" << setw(14) << "Speed" << setw(14) << "Distance" << setw(14) << "Total Time" << setw(14) << "Fuel" << setw(14) << "Oxygen" << setw(11) << "Resupply" << setw(14) << "Food" << setw(11) << "Resupply" << setw(15) << "Score" << endl;
  cout << scientific << setprecision(5);
  double travelTime = 0.0;
  double travelDistance = 0.0;
  //double ucost = 0.0;
  int pathLength = path.size();
  for (int i = 1; i < pathLength; i++)
  {
    int node = path[i];
    PlanetData planet = planets[node];
    double d = nodeDistances[path[i-1]][node], v = speeds[i], t = d / v;
    //ucost += calculateCost(node0, node);
    travelTime += t;
    travelDistance += d;
    double fu = fuel[i], o = oxygen[i], fo = food[i], s = travelTime - (fu + o + fo);
    string o2 = "-", fo2 = "-";
    switch (planet.resupply)
    {
      case OXYGEN:
        o2 = "-->  " + to_string((int) maxOxygen);
        break;
      case FOOD:
        fo2 = "-->  " + to_string((int) maxFood);
        break;
      default:
        break;
    }
    if (edgeInHostileRadius(path[i-1], node))
      cout << "Edge crosses hostile radius" << endl;
    cout << i << ") "
         << right
         << setw(11) << planet.name << "   "
         << setw(11) << v << "   "
         << setw(11) << d << "   "
         << setw(11) << travelTime << "   "
         << setw(11) << fu << "   "
         << setw(11) << o << "  "
         << setw(9) << o2 << "   "
         << setw(11) << fo << "  "
         << setw(9) << fo2 << "   "
         << setw(12) << s
         << endl;
  }
  cout << endl << "Final stats: " << endl;
  double fuelLeft = fuel.back(), oxygenLeft = oxygen.back(), foodLeft = food.back();
  cout << "Total time: " << travelTime << endl;
  cout << "Total Distance: " << travelDistance << endl;
  //cout << "Unconstained Score: " << ucost << endl;
  cout << "Remaining fuel: " << fuelLeft << endl;
  cout << "Remaining oxygen: " << oxygenLeft << endl;
  cout << "Remaining food: " << foodLeft << endl;
  cout << "Score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

void testPath()
{
  string input;
  cout << endl << "Path (New): ";
  getline(cin, input);
  cout << endl;
  int commaCount = count(input.begin(), input.end(), ',');
  vector<int> path;
  for (int i = 0; i < commaCount; i++)
  {
    int commaIndex = input.find(',');
    string planet = trim(input.substr(0, commaIndex));
    input = input.substr(commaIndex + 1);
    for (int i = 0; i < planets.size(); i++)
    {
      if (planets[i].name != planet)
        continue;
      path.push_back(i);
      break;
    }
  }
  string planet = trim(input);
  for (int i = 0; i < planets.size(); i++)
  {
    if (planets[i].name != planet)
      continue;
    path.push_back(i);
    break;
  }
  int pathLength = commaCount + 1;
  vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength);
  if (optimizePath(path, speeds, fuel, oxygen, food) == 0)
    printData(path, speeds, fuel, oxygen, food);
}

void oldTestPath()
{
  string input, input2;
  cout << endl << "Path (Old): ";
  getline(cin, input);
  cout << "Speeds: ";
  getline(cin, input2);
  int commaCount = count(input.begin(), input.end(), ',');
  vector<int> path;
  vector<double> speeds;
  for (int i = 0; i < commaCount; i++)
  {
    int commaIndex = input.find(',');
    string planet = trim(input.substr(0, commaIndex));
    input = input.substr(commaIndex + 1);
    for (int i = 0; i < planets.size(); i++)
    {
      if (planets[i].name != planet)
        continue;
      path.push_back(i);
      break;
    }
    commaIndex = input2.find(',');
    speeds.push_back(stod(trim(input2.substr(0, commaIndex))));
    input2 = input2.substr(commaIndex + 1);
  }
  speeds.push_back(stod(trim(input2)));
  string planet = trim(input);
  for (int i = 0; i < planets.size(); i++)
  {
    if (planets[i].name != planet)
      continue;
    path.push_back(i);
    break;
  }

  cout << "Name of map used: " << mapName << endl;
  cout << "Planet travel order (all values rounded to 4 decimals and in AU): " << endl << endl;
  cout << right << setw(14) << "Planet: " << setw(14) << "Speed: " << setw(14) << "Dist: " << setw(14) << "Total Time: " << setw(14) << "Fuel: " << setw(14) << "Oxyg: " << setw(14) << "Food: " << setw(15) << "Score: " << endl;
  cout << fixed;
  int pathLength = path.size();
  vector<array<double, 3>> pathData;
  int node0 = 0;
  double travelTime = 0.0;
  for (int i = 0; i < pathLength; i++)
  {
    int node = path[i];
    PlanetData planet = planets[node];
    array<double, 3> previousNode;
    if (i != 0)
      previousNode = pathData[i - 1];
    else
    {
      previousNode = array<double, 3>();
      previousNode[0] = maxFuel;
      previousNode[1] = maxOxygen;
      previousNode[2] = maxFood;
    }
    double v = speeds[i], t, o, fo, d = calculateDistance(planets[node0].position, planet.position);
    switch (planet.resupply)
    {
      case OXYGEN:
        t = d / v;
        o = maxOxygen;
        fo = previousNode[2] - kl * t;
        break;
      case FOOD:
        t = d / v;
        fo = maxFood;
        o = previousNode[1] - kl * t;
        break;
      default:
        t = d / v;
        o = previousNode[1] - kl * t, fo = previousNode[2] - kl * t;
        break;
    }
    double fu = previousNode[0] - kf * d * v * v;
    travelTime += t;
    double s = travelTime - (fu + o + fo);
    array<double, 3> tnode = array<double, 3>();
    tnode[0] = fu;
    tnode[1] = o;
    tnode[2] = fo;
    pathData.push_back(tnode);
    if (edgeInHostileRadius(node0, node))
      cout << "Edge crosses hostile radius" << endl;
    cout << i + 1 << ") "
         << right
         << setw(11) << planet.name << "   "
         << setw(11) << setprecision(9) << v << "   "
         << setw(11) << setprecision(6) << d << "   "
         << setw(11) << travelTime << "   "
         << setw(11) << fu << "   "
         << setw(11) << o << "   "
         << setw(11) << fo << "   "
         << setw(12) << s
         << endl;
    switch (planet.resupply)
    {
      case OXYGEN:
        cout << right << setw(73) << "(" << (int) maxOxygen << "-Resupp)" << endl;
        break;
      case FOOD:
        cout << right << setw(87) << "(" << (int) maxFood << "-Resupp)" << endl;
        break;
      default:
        break;
    }
    node0 = node;
  }
  cout << endl << "Final stats: " << endl;
  array<double, 3> lastNode = pathData[pathLength - 1];
  double fuelLeft = lastNode[0], oxygenLeft = lastNode[1], foodLeft = lastNode[2];
  cout << "Travel time: " << travelTime << endl;
  cout << "Remaining fuel: " << fuelLeft << endl;
  cout << "Remaining oxygen: " << oxygenLeft << endl;
  cout << "Remaining food: " << foodLeft << endl;
  cout << "Final score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

/*// Searches list of best paths to find final optimal solution
int searchBestPaths(double minCost, double maxCost)
{
    TreeNode head = TreeNode(lastNodeIndex, NULL, 0.0);
    cout << "Before tree" << endl;
    findPathTree(&head, minCost, maxCost);
    cout << "After tree" << endl;
    int pathCount = possiblePaths.size(), validCount = 0;
    BestPath bestPath;
    // Indicate to compiler to use parallel processors for search
#pragma omp declare reduction(min_idx:BestPath : omp_out = (omp_in.score < omp_out.score) ? omp_in : omp_out) \
        initializer(omp_priv = BestPath())
#pragma omp parallel for reduction(min_idx : bestPath) reduction(+ : validCount)
    
    for (int i = 0; i < pathCount; i++)
    {
      TreeNode *p = possiblePaths[i]->parent;
      vector<int> path;
      while (p)
      {
        path.push_back(p->nodeIndex);
        p = p->parent;
      }
      int pathLength = path.size();
      if (pathLength > 0)
      {
        vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength);
        if (optimizePath(path, speeds, fuel, oxygen, food) == 0)
        {
          validCount++;
          double score = 0.0;
          int node0 = 0;
          for (int j = 0; j < pathLength; j++)
          {
            int node = path[j];
            score += nodeDistances[node0][node] / speeds[j];
            node0 = node;
          }
          score -= (fuel.back() + oxygen.back() + food.back());
          if (score < bestPath.score)
          {
            bestPath.index = i;
            bestPath.path = path;
            bestPath.score = score;
            bestPath.speeds = speeds;
            bestPath.fuel = fuel;
            bestPath.oxygen = oxygen;
            bestPath.food = food;
          }
        }
      }
    }
    if (bestPath.score < INF)
    {
      //cout << "found " << validCount << " valid out of " << pathCount << " paths, best is path: " << bestPath.index << endl;
      printData(bestPath.path, bestPath.speeds, bestPath.fuel, bestPath.oxygen, bestPath.food);
      return 0;
    }
    else
      return -1;
}*/

int main(int argc, char *argv[])
{
  // Command line options
  int opt, index;
  static struct option long_options[] = 
  {
    {"map", required_argument, 0, 'm'}, // Input map
    {"fuel", required_argument, 0, 'f'},
    {"oxygen", required_argument, 0, 'o'},
    {"food", required_argument, 0, 'd'},
    {"maxPaths", required_argument, 0, 's'}, // Max number of best paths to generate
    {"pathFactor", required_argument, 0, 'g'}, // nodeCount * pathFactor = paths to generate, capped at maxPaths
    {"bidirection", required_argument, 0, 'b'}, // Use bidirectional A* search
    {"test", required_argument, 0, 't'}, // Test path
    {"scank", required_argument, 0, 'k'}, // Scan for best slope during speed optimization
    {"ideal", required_argument, 0, 'e'}, // Use precalculated ideal speeds
    {"nproc", required_argument, 0, 'n'}, // Use n parallel processors
    {"skipm1", required_argument, 0, 'r'}, // Skip speed optimization method 1
    {"skipm2", required_argument, 0, 'p'}, // Skip speed optimization method 2
    {"skipm3", required_argument, 0, 'q'}, // Skip speed optimization method 3
    {"skipSingle", required_argument, 0, 'w'}, // Skip single search
    {"skipOld", required_argument, 0, 'l'}, // Skip old optimization methods
    {"oldYens", required_argument, 0, 'y'}, // Use old Yens algorithm
    {0, 0, 0, 0}
  };
  while ((opt = getopt_long(argc, argv, "m:f:o:d:s:g:b:t:k:e:n:r:p:q:w:l", long_options, &index)) != -1)
  {
    switch (opt)
    {
      case 'm':
        fileName = optarg;
        break;
      case 'f':
        maxFuel = atof(optarg);
        break;
      case 'o':
        maxOxygen = atof(optarg);
        break;
      case 'd':
        maxFood = atof(optarg);
        break;
      case 's':
        maxPaths = atoi(optarg);
        break;
      case 'g':
        pathFactor = atoi(optarg);
        break;
      case 'b':
        useBidirection = atoi(optarg);
        break;
      case 't':
        testMode = atoi(optarg);
        break;
      case 'k':
        scanSlope = atoi(optarg);
        break;
      case 'e':
        idealCheck = atoi(optarg);
        break;
      case 'n':
        nproc = atoi(optarg);
        break;
      case 'r':
        skipM1 = atoi(optarg);
        break;
      case 'p':
        skipM2 = atoi(optarg);
        break;
      case 'q':
        skipM3 = atoi(optarg);
        break;
      case 'w':
        skipSingle = atoi(optarg);
        break;
      case 'l':
        skipOld = atoi(optarg);
        break;
      case 'y':
        useOldYens = atoi(optarg);
        break;
      default:
        break;
    }
  }
  
  cout << endl << "Welcome Mission Control!" << endl;
  if (fileName.empty())
  {
    cout << "Use these instructions to begin stress-testing:" << endl;
    cout << endl;
    cout << "This program expects a map CSV file formatted as per the challenge guide (Planet Name,x,y,type,resupply,hostile_radius)." << endl;
    cout << "It will output the best route from the first planet in the file (right after the header line) to the planet on the last line of the file." << endl;
    cout << endl;
    cout << "Put the CSV file in the same directory/folder as this program and then simply input the file name below (e.g., ExampleMap.csv)." << endl;
    cout << "If the map cannot be placed in the same folder, the full file path can be provided:" << endl;
    cout << "e.g., C:\\Users\\Your Username\\Folder Name\\ExampleMap.csv on Windows or /Users/Your Username/Folder Name/ExampleMap.csv on Mac." << endl;
  }

#ifdef _OPENMP
  if (nproc > 0) 
    omp_set_num_threads(nproc);
  int np = omp_get_num_threads();
  //cout << endl << "Parallel processors: inputted " << nproc << ", using " << np << endl;
#endif

  // Input and setup
  int error;
  do
  {
    planets.clear();
    error = parseCSV();
    fileName = "";
  } while (error != 0);
  initializeLists();

  // Pathfinding and speed/resource optimization
  /*tartTime = chrono::steady_clock::now();
  for (int i = 0; i < 1000000; i++) aStar();
  long long c = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
  cout << "Calculation time (milliseconds): " << fixed << setprecision(3) << c / 1000.0 << endl;*/
  vector<vector<int>> bestPaths;
  int pathCount = pathFactor * nodeCount;
  if (pathCount > maxPaths) pathCount = maxPaths;
  else if (pathCount < 50) pathCount = 50;
  if (useOldYens == 1)
  {
    if (oldYens(bestPaths, pathCount) == -1)
    {
      cout << "Mission failed: Valid path could not be found. Please try again." << endl;
      return -1;
    }
  }
  else
  {
    if (yens(bestPaths, pathCount) == -1)
    {
      cout << "Mission failed: Valid path could not be found. Please try again." << endl;
      return -1;
    }
  }
  double bestScore = INF;
  vector<int> bestPath;
  vector<double> bestSpeeds, bestFuel, bestOxygen, bestFood;
  //int ipath = 0;
  for (auto &path : bestPaths)
  {
    int pathLength = path.size();
    if (pathLength == 0)
      continue;
    vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength);
    fuel[0] = maxFuel, oxygen[0] = maxOxygen, food[0] = maxFood;
    if (optimizePath(path, speeds, fuel, oxygen, food) == 0)
    {
      double score = 0.0;
      for (int i = 1; i < pathLength; i++)
      {
        double t = nodeDistances[path[i-1]][path[i]] / speeds[i];
        score += t;
      }
      if (pathLength > 1)
      {
        score -= (fuel.back() + oxygen.back() + food.back());
        //printPath("path "+to_string(ipath), path, score);
      } 
      else
        score = INF;
      if (score < bestScore)
      {
        bestScore = score;
        bestPath = path;
        bestSpeeds = speeds;
        bestFuel = fuel;
        bestOxygen = oxygen;
        bestFood = food;
      }
    }
    //ipath++;
  }
  if (bestScore < INF)
    printData(bestPath, bestSpeeds, bestFuel, bestOxygen, bestFood);
  else
  {
    cout << "Mission failed: Valid path could not be found. Please try again." << endl;
    return -1;
  }

  long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
  double milliseconds = calcTime / 1000.0;
  cout << endl << "Calculation time: " << fixed << setprecision(4) << milliseconds / 1000 << " seconds (" << setprecision(3) << milliseconds << " milliseconds)" << endl << endl;

  while (testMode > 0)
  {
    oldTestPath();
    testPath();
  }

  /*long long calcTime;
  if (bestPath.length() > 0)
  {
    vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength);
    if (skipSingle == 0)
    {
      cout << endl;
      cout << "Single search: " << endl;
      if (optimizePath(path, speeds, fuel, oxygen, food) == 0)
        printData(path, speeds, fuel, oxygen, food);
      else
        cout << "Invalid path" << endl;
      // Convert microseconds to milliseconds, fix 3 decimal places
      calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
      cout << "Calculation time (milliseconds): " << fixed << setprecision(3) << calcTime / 1000.0 << endl;
      cout << endl;
    }
    if (skipOld == 0)
    {
      cout << endl;
      cout << "Old single search: " << endl;
      if (oldOptimizePath(path, speeds, fuel, oxygen, food) == 0)
        printData(path, speeds, fuel, oxygen, food);
      else
        cout << "Invalid path" << endl;
      // Convert microseconds to milliseconds, fix 3 decimal places
      calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
      cout << "Calculation time (milliseconds): " << fixed << setprecision(3) << calcTime / 1000.0 << endl;
      cout << endl;
    }
  }
  else
  {
    cout << "Path has length 0" << endl;
  }

  // Speed/resource optimization
  if (searchThreshold > 0)
  {
    cout << endl;
    double minCost = 0.0, maxCost = nodes.back().minCost * searchThreshold;
    // Smaller maxCost improves efficiency but lower accuracy, so if there is a valid best path within a smaller interval, it is better to use it
    // If not, iterate through increasing cost intervals until valid best path is found
    while (true)
    {
      cout << "iteration" << endl;
      try
      {
        if (searchBestPaths(minCost, maxCost) == 0)
          break;
        minCost = maxCost;
        maxCost *= 2;
      }
      catch (...)
      {
        // If error is thrown (eg. maxCost outside double limit) before valid path is found, the map is invalid
        cout << "Mission failed: Valid path could not be found. Please try again." << endl;
        return -1;
      }
    }
  }
  */
}
