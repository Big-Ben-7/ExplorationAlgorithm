#include <iostream>
#include <fstream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <getopt.h>
#include <array>
#include <set>
using namespace std;

#define kf 0.15
#define kl 0.3
#define MAX_FUEL 3000
#define MAX_OXYGEN 2000
#define MAX_FOOD 2000
#define INF numeric_limits<double>::infinity()
#define EPSILON std::numeric_limits<double>::epsilon()

// Command line options
int maxPaths = 2000;
int pathFactor = 5;
double distanceScale = 1.0;
int testMode = 0;
string fileName = "";

struct Position
{
  double x, y;
  Position()
  {
    x = y = 0;
  }
  Position(double newX, double newY)
  {
    x = newX, y = newY;
  }
};
enum Resupply
{
  NONE,
  OXYGEN,
  FOOD,
  BOTH
};
// Represents a planet on the map
struct PlanetData
{
  string name;
  Position position;
  Resupply resupply;
  double hostileRadius;
  PlanetData(string n, double x, double y, Resupply r, double h)
  {
    name = n, position = Position(x, y), resupply = r, hostileRadius = h;
  }
};
// Represents path of nodes (planets)
struct Path
{
  vector<int> path;
  double cost;
  Path(vector<int> p, double c)
  {
    path = p;
    cost = c;
  }
  // Override comparison operator for Yen's algorithm set
  bool operator<(const Path &other) const
  {
    if (fabs(cost - other.cost) < EPSILON)
      return path < other.path; // Allow different paths with same cost
    return cost < other.cost;   // Sort by cost in Yen's algorithm set
  }
};
// Used to test different combinations of speeds in speed optimization
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
// Represents constraint boundary curve in speed optimization
struct Boundary
{
  double xb, yb;
  Boundary(double x, double y)
  {
    xb = x;
    yb = y;
  }
};

// Adjacency list and 3 adjacency matrix structures used to efficiently represent graph
vector<vector<int>> neighbors; // Contains list of adjacent nodes for each node
vector<vector<double>> nodeCosts;
vector<vector<double>> nodeDistances;
vector<vector<double>> initialCosts; // Stores initial cost between two node indices to restore changes to nodeCosts in Yen's algorithm

vector<double> hF, hB; // List of each node's heuristics for forward and backward search in pathfinding
vector<PlanetData> planets;
int nodeCount, lastNodeIndex, longestNameLength;
chrono::steady_clock::time_point startTime;
string mapName;

// Golden section ratio for Brent's minimization
const double phi = (3 - sqrt(5)) / 2;
// Speed optimization by first derivative test: cost = c(x) = time + fuel + food + oxygen = d/v + kf*d*v^2 + kl*d/v + kl*d/v
// Cost minimized at c'(x) = 0 => (-d + 2*kf*d*v^3 - kl*d - kl*d)/v^2 = 0 => v = cbrt((1 + kl+kl) / (2*kf))
const double idealSpeed = cbrt((1 + 2 * kl) / (2 * kf));
// With one resupply type remaining, ideal speed can disregard consumption of one resource type (one kl term) because it will be resupplied later
const double singleResupplySpeed = cbrt((1 + kl) / (2 * kf));
// With both resupply type remaining, ideal speed can disregard consumption of both kl terms because both oxygen and food will be resupplied later
const double doubleResupplySpeed = 1 / cbrt(2 * kf);

/************************************************************ Input ************************************************************/

string toLower(const string &str)
{
  string str2 = str;
  for (auto &c : str2)
    c = static_cast<unsigned char>(tolower(static_cast<unsigned char>(c)));
  return str2;
}

string trim(const string &str)
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

bool isValidPlanet(const string &planetName)
{
  bool alphanumeric = false;
  for (auto &c : planetName)
  {
    if (isalnum(c))
    {
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
  // Planet names can't be all special characters
  return alphanumeric;
}

double calculateDistance(const Position &pos1, const Position &pos2)
{
  double dx = pos2.x - pos1.x, dy = pos2.y - pos1.y;
  return sqrt(dx * dx + dy * dy);
}

// Extracts data from inputted CSV map with error handling
int parseCSV()
{
  vector<int> hostileIndices;
  if (fileName.empty())
  {
    cout << endl
         << "Input map file: ";
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
      else if (hostileRadius < EPSILON)
      {
        cout << "Error: The hostile radius (" << hostileRadius << ") of hostile planet " << planetName << " (entry " << lineNumber << ") must be positive." << endl;
        return -1;
      }
      hostileIndices.push_back(lineNumber - 2);
    }
    if (planetName.size() > longestNameLength)
      longestNameLength = planetName.size();
    planets.emplace_back(planetName, x, y, resupplyEnum, hostileRadius);
  }

  if (lineNumber < 3)
  {
    cout << "Error: There must be at least 2 planets in the map." << endl;
    return -1;
  }
  if (planets.back().hostileRadius != 0)
  {
    cout << "Error: The target planet (" << planets.back().name << ") cannot be hostile." << endl;
    return -1;
  }
  nodeCount = lineNumber - 1, lastNodeIndex = lineNumber - 2;

  // Remove unreachable planets (lying inside hostile radius)
  vector<int> unreachableIndices;
  int hostileCount = hostileIndices.size();
  for (int i : hostileIndices)
  {
    PlanetData planet = planets[i];
    Position pos = planet.position;
    double hostileRadius = planet.hostileRadius;
    for (int j = 0; j < nodeCount; j++)
    {
      PlanetData planet2 = planets[j];
      if (planet2.hostileRadius < EPSILON && calculateDistance(pos, planet2.position) <= hostileRadius)
      {
        unreachableIndices.push_back(j);
        if (j == 0)
        {
          cout << "Error: The first planet (" << planets[0].name << ") is in the hostile radius of planet " << planet.name << "." << endl;
          return -1;
        }
        else if (j == lastNodeIndex)
        {
          cout << "Error: The target planet (" << planets.back().name << ") is in the hostile radius of planet " << planet.name << "." << endl;
          return -1;
        }
      }
    }
  }
  sort(unreachableIndices.begin(), unreachableIndices.end(), greater<int>());
  for (int i : unreachableIndices)
    planets.erase(planets.begin() + i);
  nodeCount = planets.size();
  lastNodeIndex = nodeCount - 1;

  if (fabs(distanceScale - 1) > EPSILON)
  {
    for (auto &planet : planets)
    {
      Position &pos = planet.position;
      pos.x *= distanceScale;
      pos.y *= distanceScale;
      planet.hostileRadius *= distanceScale;
    }
  }
  iFile.close();
  return 0;
}

/************************************************************ Pathfinding ************************************************************/

// Returns cost (time + resource consumption) of edge connecting two nodes
double calculateCost(const int node1, const int node2)
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

bool edgeInHostileRadius(const int startNode, const int endNode)
{
  // Calculates intersection of line through (x1,y1) and (y1,y2) with circle of radius r at (x3,y3)
  Position &start = planets[startNode].position, end = planets[endNode].position;
  // Line eq. (parametric): x = x1 + cos(theta)*d, y = y1 + sin(theta)*d where d = distance along line and theta = atan(slope)
  double x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y, rise = y2 - y1, run = x2 - x1;
  double theta = atan2(rise, run), cosTheta = cos(theta), sinTheta = sin(theta); // Precompute trig for efficiency
  double l = run * run + rise * rise;                                            // Length of line
  for (auto &planet : planets)
  {
    if (planet.hostileRadius < EPSILON)
      continue;
    Position &center = planet.position;
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

// Provides pathfinding setup
void initializeLists()
{
  // Heuristic constant for A* pathfinding (minimum cost per unit distance traveled)
  double heuristic = (1 + kl) / singleResupplySpeed + kf * singleResupplySpeed * singleResupplySpeed;
  hF = vector<double>(nodeCount, 0), hB = vector<double>(nodeCount, 0);
  Position sourcePos = planets[0].position, sinkPos = planets[lastNodeIndex].position;
  neighbors = vector<vector<int>>(nodeCount);
  nodeCosts = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  nodeDistances = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  initialCosts = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  for (int i = 0; i < nodeCount; i++)
  {
    double hostileRadius = planets[i].hostileRadius;
    for (int j = i + 1; j < nodeCount; j++)
    {
      // Eliminate edges crossing hostile radius
      if (hostileRadius != 0 || planets[j].hostileRadius != 0 || edgeInHostileRadius(i, j))
      {
        nodeCosts[i][j] = initialCosts[i][j] = nodeCosts[j][i] = initialCosts[j][i] = INF;
        if (j == lastNodeIndex)
          hF[i] = heuristic * calculateDistance(planets[i].position, sinkPos);
        if (i == 0)
          hB[j] = heuristic * calculateDistance(planets[j].position, sourcePos);
        continue;
      }
      double distance = nodeDistances[i][j] = nodeDistances[j][i] = calculateDistance(planets[i].position, planets[j].position);
      neighbors[i].push_back(j);
      neighbors[j].push_back(i);
      nodeCosts[i][j] = initialCosts[i][j] = calculateCost(i, j);
      nodeCosts[j][i] = initialCosts[j][i] = calculateCost(j, i);

      // A* heuristic for each node
      if (j == lastNodeIndex)
        hF[i] = heuristic * distance; // Use Euclidian distance to target as heuristic for forward search
      if (i == 0)
        hB[j] = heuristic * distance; // Use Euclidian distance to source as heuristic for backward search
    }
    nodeCosts[i][i] = initialCosts[i][i] = INF;
  }
}

// Implements modified A* algorithm with bidirectional search to find single best-path from source to sink node
// Works by running a forward and backward A* search (using edge relaxation and heuristic guide) until they meet, forming a complete path
int bidirectionAStar(vector<int> &bestPath, const int source, const int sink)
{
  // Forward and backward priority queues of <cost, index> to pick most promising (least cost) node
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pqF, pqB;
  pqF.emplace(0, source);
  pqB.emplace(0, sink);
  // Forward and backward g-scores (costs), initialized to infinity
  vector<double> gF(nodeCount, INF), gB(nodeCount, INF);
  gF[source] = gB[sink] = 0;                                           // Initialize cost to source in forward and sink in backward search to 0
  vector<bool> visitedF(nodeCount, false), visitedB(nodeCount, false); // Visited nodes in forward and backward search
  vector<int> parentsF(nodeCount, -1), parentsB(nodeCount, -1);        // Each node's previous node in path (for path reconstruction)

  double bestCost = INF;
  int meetingNode = -1;
  while (!pqF.empty() && !pqB.empty())
  {
    const pair<double, int> &forwardPair = pqF.top(), backwardPair = pqB.top();
    double costF = forwardPair.first, costB = backwardPair.first;
    // Stop when sum of forward and backward costs exceeds best path cost (best meeting point has been found)
    if (bestCost < INF)
    {
      if (costF + costB >= bestCost)
        break;
    }
    // Forward search
    pqF.pop();
    int node1 = forwardPair.second;
    if (!visitedF[node1])
    {
      visitedF[node1] = true;
      double g1 = gF[node1];
      vector<int> &nodeNeighbors = neighbors[node1];
      for (int node2 : nodeNeighbors) // Iterate through adjacent nodes
      {
        double cost = nodeCosts[node1][node2];
        if (cost == INF || visitedF[node2])
          continue;
        double newG = g1 + cost;
        if (newG < gF[node2])
        {
          // Relax edge
          gF[node2] = newG;
          parentsF[node2] = node1;
          pqF.emplace(newG + hF[node2], node2);
          if (gB[node2] < INF)
          {
            double newCost = gF[node2] + gB[node2];
            if (newCost < bestCost)
            {
              bestCost = newCost;
              meetingNode = node2;
            }
          }
        }
      }
    }
    // Backward search
    pqB.pop();
    node1 = backwardPair.second;
    if (!visitedB[node1])
    {
      visitedB[node1] = true;
      double g1 = gB[node1];
      vector<int> &nodeNeighbors = neighbors[node1];
      for (int node2 : nodeNeighbors) // Iterate through adjacent nodes
      {
        double cost = nodeCosts[node2][node1];
        if (cost == INF || visitedB[node2])
          continue;
        double newG = g1 + cost;
        if (newG < gB[node2])
        {
          // Relax edge
          gB[node2] = newG;
          parentsB[node2] = node1;
          pqB.emplace(newG + hB[node2], node2);
          if (gF[node2] < INF)
          {
            double newCost = gF[node2] + gB[node2];
            if (newCost < bestCost)
            {
              bestCost = newCost;
              meetingNode = node2;
            }
          }
        }
      }
    }
  }

  // Reconstruct path
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
  if (meetingNode != sink && (rightPath.empty() || rightPath.back() != sink))
    return -1; // Path didn't connect to sink
  bestPath = leftPath;
  bestPath.insert(bestPath.end(), rightPath.begin(), rightPath.end());
  return bestPath.empty() ? -1 : 0;
}

// Implements modified Yen's algorithm to find pathCount best paths
// Works by repeatedly calling bidirectionAStar after modifying graph (blocking root path of previous best paths)
int yens(vector<vector<int>> &bestPaths, int pathCount)
{
  vector<int> bestPath, sameRootPaths, rootPath, spurPath, totalPath;
  int rv;
  rv = bidirectionAStar(bestPath, 0, lastNodeIndex);
  if (rv == -1)
    return -1;
  bestPaths.push_back(bestPath); // Find best path from source (Earth) to sink (Target)
  set<Path> potentialPaths;      // Use set to avoid duplicates and automatic sorting
  for (int k = 1; k < pathCount; k++)
  {
    vector<int> &lastBestPath = bestPaths.back();
    int n = lastBestPath.size() - 1;
    for (int i = 0; i < n; i++)
    {
      rootPath.assign(lastBestPath.begin(), lastBestPath.begin() + i + 1); // Sequence of nodes from source to spur node
      sameRootPaths.clear();                                               // Store paths with removed edges to speed up restoring
      int bestPathsCount = bestPaths.size();
      // Modify graph to produce different best path
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
        int node = lastBestPath[j];
        for (int m = 0; m < nodeCount; m++)
          nodeCosts[node][m] = nodeCosts[m][node] = INF; // Remove all nodes in root path from graph to prevent cycles
      }
      int spurNode = lastBestPath[i]; // Retrieve spur node from previous shortest path
      spurPath.clear();
      int error = bidirectionAStar(spurPath, spurNode, lastNodeIndex); // Calculate spur path from spur node to sink

      // Restore removed edges/nodes
      int sameRootCount = sameRootPaths.size();
      for (int j = 0; j < sameRootCount; j++)
      {
        vector<int> &path = bestPaths[sameRootPaths[j]];
        int node1 = path[i], node2 = path[i+1];
        nodeCosts[node1][node2] = initialCosts[node1][node2];
      }
      for (int j = 0; j < i; j++)
      {
        int node = lastBestPath[j];
        for (int m = 0; m < nodeCount; m++)
        {
          nodeCosts[node][m] = initialCosts[node][m];
          nodeCosts[m][node] = initialCosts[m][node];
        }
      }
      
      if (error == -1)
        continue;
      totalPath = rootPath;
      totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end()); // Combine root and spur path into complete path
      int pathLength = totalPath.size();
      double cost = 0;
      for (int j = 1; j < pathLength; j++)
        cost += initialCosts[totalPath[j - 1]][totalPath[j]];
      potentialPaths.emplace(totalPath, cost); // Adds complete path sorted by ascending cost, avoiding duplicates
    }

    if (potentialPaths.empty()) // No spur paths (e.g. all already exhausted or none exist)
      break;
    auto it = potentialPaths.begin();
    const Path &path = *it; // First element of set returns best path
    bestPaths.push_back(path.path);
    potentialPaths.erase(it);
  }
  return 0;
}

/************************************************************ Speed/Resource optimization ************************************************************/

// Bisection algorithm to find root c of func in [a, b], works by repeatedly halving interval until func(c) is approximately 0
double bisection(double a, double b, const double x0, const double y0, 
                 double (*func)(const double, const double, const double), const double tolerance = 1e-6)
{
  double c;
  c = 0.5 * (a + b);
  double fa = func(a, x0, y0);
  double fb = func(b, x0, y0);
  while (fabs(b - a) > tolerance * c)
  {
    double fc = func(c, x0, y0);
    if (fabs(fc) < tolerance)
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

// Equations to find intersection of boundary curves
double func(const double theta, const double x0, const double y0)
{
  double cosTheta = cos(theta), sinTheta = sin(theta);
  return x0 * sinTheta + y0 * cosTheta - sinTheta * cosTheta;
}
double derivFunc(const double theta, const double x0, const double y0)
{
  double cosTheta = cos(theta), sinTheta = sin(theta);
  return x0 * cosTheta - y0 * sinTheta - (cosTheta * cosTheta - sinTheta * sinTheta);
}

// Optimizes speed solution along line of slope m (see optimizeSpeed())
int getIdealSpeed(double &v1, double &v2, double &score, const double m,
                  const double b, const double c, const vector<Boundary> &bs)
{
  double outerIntersection = 1 / sqrt(1 + m * m); // Intersection of line and circle (fuel constraint)
  double innerIntersection = 0.0;
  double xi, x1, y1;
  for (auto &boundary : bs) // Intersection of line and food/oxygen boundary curves
  {
    x1 = boundary.xb;
    y1 = boundary.yb;
    if (fabs(x1) < EPSILON)
      xi = y1 / m;
    else if (fabs(y1) < EPSILON)
      xi = x1;
    else
      xi = x1 + y1 / m;
    if (xi > innerIntersection)
      innerIntersection = xi;
  }
  if (innerIntersection >= outerIntersection)
    return -1;
  double v1Ideal = cbrt(0.5 * b / c); // Analytic solution of ideal speed on line
  double di = 1e-5 * (outerIntersection - innerIntersection);
  double v1Min = innerIntersection + di, v1Max = outerIntersection - di;
  int returnVal;
  // Clamps solution to be in the area bounded by constraint curves
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

// Uses Brent's minimization method (parabolic interpolation with golden section search fallback) to find best speeds (see optimizeSpeed())
int brentMinimize(double &xm, double &fm, double &v1, double &v2, double a, double b,
                  const double fuel0, const double mvx, const double mvy, const vector<Boundary> &bs,
                  const double tolerance = 1e-6, const int maxIter = 500)
{
  // Initialize bracketing points
  double bb, cc, s;
  int returnVal;
  double x = a + phi * (b - a);
  s = tan(x);
  bb = mvx + mvy / s;
  cc = fuel0 + fuel0 * s * s;
  double fx;
  returnVal = getIdealSpeed(v1, v2, fx, s, bb, cc, bs);
  if (returnVal < 0)
    fx = INF;
  double w = x, fw = fx; // w is previous best point
  double v = x, fv = fx; // v is previous of previous best point
  double d = 0.0;        // Current step
  double e = 0.0;        // Distance moved from previous step

  for (int iter = 0; iter < maxIter; iter++)
  {
    double midpoint = 0.5 * (a + b);
    double tol1 = tolerance * fabs(x);
    double tol2 = 2.0 * tol1;

    // Stop when interval sufficiently small
    if (fabs(x - midpoint) <= (tol2 - 0.5 * (b - a)))
    {
      xm = x;
      fm = fx;
      return 0;
    }

    // Try parabolic interpolation
    if (fabs(e) > tol1)
    {
      // Fit parabola through (w, fw), (x, fx), (v, fv)
      double r = (x - w) * (fx - fv);
      double q = (x - v) * (fx - fw);
      double p = (x - v) * q - (x - w) * r;
      q = 2.0 * (q - r);
      q > 0.0 ? p = -p : q = -q;

      // Check if parabolic step is acceptable
      if (fabs(p) < fabs(0.5 * q * e) && p > q * (a - x) && p < q * (b - x))
      {
        // Take parabolic step
        d = p / q;
        double u = x + d;

        // f cannot be evaluated too close to a or b
        if ((u - a) < tol2 || (b - u) < tol2)
          d = (x < midpoint) ? tol1 : -tol1;
      }
      else
      {
        // Fall back to golden section search
        e = (x >= midpoint) ? (a - x) : (b - x);
        d = phi * e;
      }
    }
    else
    {
      // Fall back to Golden section search
      e = (x >= midpoint) ? (a - x) : (b - x);
      d = phi * e;
    }

    // Evaluate function at new point
    double u = (fabs(d) >= tol1) ? (x + d) : (x + ((d > 0.0) ? tol1 : -tol1));
    double fu;
    s = tan(u);
    bb = mvx + mvy / s;
    cc = fuel0 + fuel0 * s * s;
    returnVal = getIdealSpeed(v1, v2, fu, s, bb, cc, bs);
    if (returnVal < 0)
      fu = INF;
    // Update bracketing triplet
    if (fu <= fx)
    {
      u >= x ? a = x : b = x;
      v = w, fv = fw;
      w = x, fw = fx;
      x = u, fx = fu;
    }
    else
    {
      u < x ? a = u : b = u;
      if (fu <= fw || w == x)
      {
        v = w, fv = fw;
        w = u, fw = fu;
      }
      else if (fu <= fv || v == x || v == w)
      {
        v = u;
        fv = fu;
      }
    }
  }

  xm = x;
  fm = fx;
  return 0;
}

// Divides path into two segments: i0-1 to im and im to i1 (i0=im for our calls) and optimizes speed for i0-1 to im
// Optimization is done by scaling ideal speeds for nodes in i0-1 to im by constant factor and for nodes in im to i1 by constant factor
// Method 3 in OptimizePath() uses a loop to set each node to im and i0, thus eventually optimizing the entire path
int optimizeSpeed(const int i0, const int im, const int i1, const vector<int> &path, const vector<double> &mb,
                  vector<double> &speeds, const vector<double> &fuel, const vector<double> &oxygen, const vector<double> &food)
{
  // Each constraint (fuel, food, oxygen) can be represented by a boundary curve on a graph that depends on the resupply type of each node
  // The area inside these boundaries is the solution space that will satisfy these constraints (resources never drop below 0)
  int returnVal = -1;
  double v1, v2, score;
  double fuel0 = fuel[i0 - 1], oxygen0 = oxygen[i0 - 1], food0 = food[i0 - 1];
  int ni = i1 - i0 + 2;
  vector<double> cs(ni), ci(ni);
  cs[0] = ci[0] = 0.0;
  double dx = 0.0, dy = 0.0, sx = 0.0, sy = 0.0, mx = 0.0, my = 0.0;
  int i;
  for (i = i0; i <= i1; i++) // Calculates values and cumulative sums used later to determine boundaries
  {
    double di = nodeDistances[path[i - 1]][path[i]];
    double ds2 = di * speeds[i] * speeds[i];
    double dos = di / speeds[i];
    if (i <= im)
    {
      dx += di;
      sx += ds2;
      mx += dos * mb[i];
    }
    else
    {
      dy += di;
      sy += ds2;
      my += dos * mb[i];
    }
    cs[i - i0 + 1] = cs[i - i0] + ds2;
    ci[i - i0 + 1] = ci[i - i0] + dos;
  }
  // The fuel constraint introduces the boundary (v1)^2 + (v2)^2 < 1
  // Graphically this is the area bounded by a circle of radius 1 (and in the first quadrant because all speeds and scale factors are positive)
  double vx = sqrt(kf / fuel0 * sx); // Constant factor for i0-1 to m is v1/vx
  double vy = sqrt(kf / fuel0 * sy); // Constant factor for m to i1 is v2/vy
  vector<Boundary> bs;
  for (int ir = 0; ir < 2; ir++) // Fills bs with list of food/oxygen boundary curves introduced by each resupply node
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
          bs.emplace_back(bx * ix, 0.0); // Vertical boundary curve
        }
        else if (bestIndex < im)
        {
          ix = ci[im - i0 + 1] - ci[bestIndex - i0 + 1];
          iy = ci[i - i0 + 1] - ci[im - i0 + 1];
          bs.emplace_back(bx * ix, by * iy); // Curvilinear boundary
        }
        else
        {
          iy = ci[i - i0 + 1] - ci[bestIndex - i0 + 1];
          bs.emplace_back(0, by * iy); // Horizontal boundary curve
        }
        bestIndex = i;
        q0 = ir == 0 ? MAX_OXYGEN : MAX_FOOD;
      }
    }
  }

  double thetaMin = 0.0, thetaMax = M_PI_2; // Angles of lines from origin to intersection points of curves (used to find bounded area)
  for (i = 0; i < bs.size(); i++)
  {
    if (fabs(bs[i].xb) < EPSILON)
    {
      if (bs[i].yb >= 1)
        thetaMin = M_PI_2;
      else
        thetaMin = max(thetaMin, asin(bs[i].yb));
    }
    else if (fabs(bs[i].yb) < EPSILON)
    {
      if (bs[i].xb >= 1)
        thetaMax = 0.0;
      else
        thetaMax = min(thetaMax, acos(bs[i].xb));
    }
    else
    {
      // Determines the intersection point of curves using root-finding algorithm (bisection)
      // Find minimum using 1st Derivative Test to determine interval of root (required for bisection)
      double tm = bisection(0, M_PI_2, bs[i].xb, bs[i].yb, derivFunc);
      thetaMin = max(thetaMin, bisection(0, tm, bs[i].xb, bs[i].yb, func));
      thetaMax = min(thetaMax, bisection(tm, M_PI_2, bs[i].xb, bs[i].yb, func));
      // Tightest thetaMin/thetaMax is used (to satisfy every constraint)
    }
    if (thetaMin >= thetaMax)
      return -1;
  }
  double a, c, s;
  double ds = (thetaMax - thetaMin) * 1e-5;
  thetaMin += ds, thetaMax -= ds;                                   // Slightly narrow boundaries for affordance
  double idealSlope = sqrt(sy / sx), idealTheta = atan(idealSlope); // Ideal theta defined when scale factors are 1
  if (idealTheta < thetaMin)
  {
    idealTheta = thetaMin; // Clamp idealTheta to thetaMin and thetaMax
    s = thetaMin + M_PI_2;
    thetaMax = min(s, thetaMax);
  }
  else if (idealTheta > thetaMax)
  {
    idealTheta = thetaMax;
    s = thetaMax - M_PI_2;
    thetaMin = max(s, thetaMin);
  }
  else
  {
    s = idealTheta - M_PI_2;
    thetaMin = max(s, thetaMin);
    s = idealTheta + M_PI_2;
    thetaMax = min(s, thetaMax);
  }

  // Iterates through lines with angles from thetaMin to thetaMax, finds best speed for point on each line within solution space
  // Minimizes score of best speeds using Brent's method to find absolute best
  double mvx = mx * vx;
  double mvy = my * vy;
  double sb = INF;
  double v1b = 0.0, v2b = 0.0;
  brentMinimize(s, sb, v1b, v2b, thetaMin, thetaMax, fuel0, mvx, mvy, bs);

  // Test solution against ideal slope as a final check (needed if min could not be found)
  idealSlope = tan(idealTheta);
  double minSlope = tan(thetaMin);
  double maxSlope = tan(thetaMax);
  double b = mvx + mvy / idealSlope;
  c = fuel0 + fuel0 * idealSlope * idealSlope;
  returnVal = getIdealSpeed(v1, v2, score, idealSlope, b, c, bs);
  if (score > sb)
  {
    v1 = v1b;
    v2 = v2b;
  }
  for (i = i0; i <= im; i++)
    speeds[i] *= v1 / vx; // Scale speeds for nodes in segment i0-1 to im by constant factor
  for (i = im + 1; i <= i1; i++)
    speeds[i] *= v2 / vy; // Scale speeds for nodes in segment im to i1 by constant factor

  return returnVal;
}

// Optimizes speed/resource usage on path with 3 methods (which increase in both robustness and computation time)
int optimizePath(const vector<int> &path, double &score, vector<double> &speeds, vector<double> &fuel, 
                 vector<double> &oxygen, vector<double> &food, vector<double> &rawOxygen, vector<double> &rawFood)
{
  int error = 0;
  bool foundOxygenResupply = false, foundFoodResupply = false;
  int pathLength = path.size();
  vector<double> mb(pathLength);

  // Method 1: set ideal speed by disregarding both, one, or none of the kl terms if node is before both, one or none of the last resupply types
  for (int i = pathLength - 1; i >= 1; i--)
  {
    // Iterate backward to find last oxygen and food resupplies
    switch (planets[path[i]].resupply)
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
  double fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
  // It can be mathematically proven (eg. with Lagrange multipliers) that the ideal speed is the same for all nodes with the same later resupply types
  score = 0.0;
  for (int i = 1; i < pathLength; i++)
  {
    int node = path[i];
    double speed = speeds[i], distance = nodeDistances[path[i - 1]][node], t = distance / speed, consumed = kl * t;
    foodLeft -= consumed, oxygenLeft -= consumed;
    if (error == 0 && (oxygenLeft < EPSILON || foodLeft < EPSILON))
      error = -1;
    rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
    fuelLeft -= kf * distance * speed * speed;
    switch (planets[node].resupply)
    {
    case OXYGEN:
      oxygenLeft = MAX_OXYGEN;
      break;
    case FOOD:
      foodLeft = MAX_FOOD;
      break;
    default:
      break;
    }
    fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
    score += t;
  }
  score -= fuel.back() + oxygen.back() + food.back();
  if (error == 0 && fuelLeft > EPSILON)
    return 0;

  // Method 2: If Method 1 (set ideal case) fails, adjust all speeds by constant factor
  double totalFuelConsumed = MAX_FUEL - fuelLeft;
  double speedFactor = sqrt(MAX_FUEL / totalFuelConsumed);
  fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
  error = 0;
  if (totalFuelConsumed >= MAX_FUEL)
  {
    score = 0.0;
    for (int i = 1; i < pathLength; i++)
    {
      int node = path[i];
      speeds[i] *= speedFactor;
      double speed = speeds[i], distance = nodeDistances[path[i - 1]][node], t = distance / speed, consumed = kl * t;
      fuelLeft -= kf * distance * speed * speed, foodLeft -= consumed, oxygenLeft -= consumed;
      if (oxygenLeft < EPSILON || foodLeft < EPSILON)
      {
        error = -1;
        break;
      }
      rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
      switch (planets[node].resupply)
      {
      case OXYGEN:
        oxygenLeft = MAX_OXYGEN;
        break;
      case FOOD:
        foodLeft = MAX_FOOD;
        break;
      default:
        break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      score += t;
    }
  }
  else
  {
    score = 0.0;
    for (int i = 1; i < pathLength; i++)
    {
      int node = path[i];
      double distance = nodeDistances[path[i - 1]][node], speed = speeds[i];
      double minOxygenSpeed = distance / (oxygenLeft / kl);
      double minFoodSpeed = distance / (foodLeft / kl);
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
      double t = distance / speed;
      double consumed = kl * t;
      fuelLeft -= kf * distance * speed * speed, oxygenLeft -= consumed, foodLeft -= consumed;
      rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
      switch (planets[node].resupply)
      {
      case OXYGEN:
        oxygenLeft = MAX_OXYGEN;
        break;
      case FOOD:
        foodLeft = MAX_FOOD;
        break;
      default:
        break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      score += t;
    }
  }
  score -= fuel.back() + oxygen.back() + food.back();
  if (error == 0)
    return 0;

  // Method 3: If Method 2 (constant factor) fails, optimize each node's speeds using a combination of algorithmic, analytical, and graphical means
  double prevScore = INF;
  // Iterate until solution is self-consistent (speed converges), guarantees correctness despite approximations
  // Capped at 10 iterations to prevent infinite loop, always converged earlier in testing
  for (int iter = 0; iter < 10; iter++)
  {
    score = 0.0;
    fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
    for (int i = 1; i < pathLength; i++)
    {
      int node = path[i];
      double distance = nodeDistances[path[i - 1]][node];
      if (i < pathLength - 1) // Iterate through each node in path and optimize speed for segment
      {
        if (optimizeSpeed(i, i, pathLength - 1, path, mb, speeds, fuel, oxygen, food) == -1)
          return -1;
      }
      double speed = speeds[i];
      double t = distance / speed;
      double consumed = kl * t;
      fuelLeft -= kf * distance * speed * speed, foodLeft -= consumed, oxygenLeft -= consumed;
      rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
      if (fuelLeft <= 0 || oxygenLeft <= 0 || foodLeft <= 0)
      {
        error = -1;
        break;
      }
      switch (planets[node].resupply)
      {
      case OXYGEN:
        oxygenLeft = MAX_OXYGEN;
        break;
      case FOOD:
        foodLeft = MAX_FOOD;
        break;
      default:
        break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      score += t;
    }
    score -= fuel.back() + oxygen.back() + food.back();
    double ds = max(0.01, fabs(score) * 5e-3);
    if (fabs(score - prevScore) < ds)
      break;
    prevScore = score;
  }
  if (fuelLeft > 0 && oxygenLeft > 0 && foodLeft > 0)
    return 0;
  else
    return -2;
}

/************************************************************ Output ************************************************************/

void printData(const vector<int> &path, const vector<double> &speeds, const vector<double> &fuel, const vector<double> &oxygen,
               const vector<double> &food, const vector<double> &rawOxygen, const vector<double> &rawFood)
{
  // Determine formatting width for each stat
  int pathLength = path.size();
  int indexWidth = (int)log10(pathLength - 1) + 1;
  if (longestNameLength < 6)
    longestNameLength = 6;
  vector<string> speedStrings;
  int longestSpeedLength = 0;
  for (int i = 0; i < pathLength; i++)
  {
    stringstream ss;
    ss << defaultfloat << setprecision(15) << speeds[i];
    string speedString = ss.str();
    speedStrings.push_back(speedString);
    if (speedString.size() > longestSpeedLength)
      longestSpeedLength = speedString.size();
  }

  // Print output
  cout << endl
       << endl
       << "PROGRAM OUTPUT" << endl;
  cout << "Map name: " << mapName << endl;
  cout << "Planet travel order from " << planets[0].name << " (excluded) to " << planets[lastNodeIndex].name << " (included)" << endl;
  cout << "Note: Some values are in scientific notation for consistent formatting (e.g., 6.67e-02 = 6.67*10^(-2) = 0.0667)" << endl
       << endl;
  cout << right << setw(longestNameLength + indexWidth + 5) << "Planet |" << setw(longestSpeedLength + 3) << "Speed To Planet |" << setw(11) << "Distance |" << setw(11) << "Time |" << setw(11) << "Fuel |" << setw(9) << "Oxygen" << setw(8) << "Res. |" << setw(9) << "Food" << setw(8) << "Res. |" << setw(12) << "Score |" << endl;
  double travelTime = 0.0;
  double travelDistance = 0.0;
  for (int i = 1; i < pathLength; i++)
  {
    int node = path[i];
    PlanetData planet = planets[node];
    double d = nodeDistances[path[i - 1]][node], v = speeds[i], t = d / v;
    travelTime += t;
    travelDistance += d;
    double fu = fuel[i], s = travelTime - (fu + oxygen[i] + food[i]);
    string o = "-", fo = "-";
    switch (planet.resupply)
    {
    case OXYGEN:
      o = to_string((int)MAX_OXYGEN);
      break;
    case FOOD:
      fo = to_string((int)MAX_FOOD);
      break;
    default:
      break;
    }
    if (edgeInHostileRadius(path[i - 1], node))
      cout << "Edge crosses hostile radius" << endl;
    cout << setw(indexWidth) << i << ")  "
         << setw(longestNameLength) << planet.name << " | "
         << setw(longestSpeedLength) << defaultfloat << setprecision(15) << v << " | "
         << setw(8) << scientific << setprecision(2) << d << " | " // Scientific notation for consistent formatting
         << setw(8) << travelTime << " | "
         << setw(8) << fu << " | "
         << setw(8) << rawOxygen[i] << "  "
         << setw(4) << o << " | "
         << setw(8) << rawFood[i] << "  "
         << setw(4) << fo << " | "
         << setw(9) << s << " |"
         << endl;
  }
  cout << endl
       << "Final stats" << endl;
  double fuelLeft = fuel.back(), oxygenLeft = oxygen.back(), foodLeft = food.back();
  cout << "Total time: " << defaultfloat << setprecision(12) << travelTime << endl;
  cout << "Total Distance: " << travelDistance << endl;
  cout << "Remaining fuel: " << fuelLeft << endl;
  cout << "Remaining oxygen: " << oxygenLeft << endl;
  cout << "Remaining food: " << foodLeft << endl;
  cout << "Score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

// Takes path input, optimizes speed and outputs score data (used to test path optimality)
int testPath()
{
  string input;
  cout << endl
       << "Path: ";
  getline(cin, input);
  if (input.empty())
    return -1;
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
  vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength), rawOxygen(pathLength), rawFood(pathLength);
  double score = INF;
  if (optimizePath(path, score, speeds, fuel, oxygen, food, rawOxygen, rawFood) == 0)
    printData(path, speeds, fuel, food, oxygen, rawOxygen, rawFood);
  return 0;
}

// Takes path and speed input, outputs raw score data (used to test data calcuation)
int rawTestPath(string &xPath, string &yPath)
{
  string input, input2;
  cout << endl
       << "Path: ";
  getline(cin, input);
  if (input.empty())
    return -1;
  cout << "Speeds: ";
  getline(cin, input2);
  if (input2.empty())
    return -1;
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
      previousNode[0] = MAX_FUEL;
      previousNode[1] = MAX_OXYGEN;
      previousNode[2] = MAX_FOOD;
    }
    double v = speeds[i], t, o, fo, d = calculateDistance(planets[node0].position, planet.position);
    switch (planet.resupply)
    {
    case OXYGEN:
      t = d / v;
      o = MAX_OXYGEN;
      fo = previousNode[2] - kl * t;
      break;
    case FOOD:
      t = d / v;
      fo = MAX_FOOD;
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
      cout << right << setw(73) << "(" << (int)MAX_OXYGEN << "-Resupp)" << endl;
      break;
    case FOOD:
      cout << right << setw(87) << "(" << (int)MAX_FOOD << "-Resupp)" << endl;
      break;
    default:
      break;
    }
    node0 = node;
  }
  cout << endl
       << "Final stats: " << endl;
  array<double, 3> lastNode = pathData[pathLength - 1];
  double fuelLeft = lastNode[0], oxygenLeft = lastNode[1], foodLeft = lastNode[2];
  cout << "Travel time: " << travelTime << endl;
  cout << "Remaining fuel: " << fuelLeft << endl;
  cout << "Remaining oxygen: " << oxygenLeft << endl;
  cout << "Remaining food: " << foodLeft << endl;
  cout << "Final score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;

  for (int i = 0; i < pathLength; i++)
  {
    xPath += "," + to_string(planets[path[i]].position.x);
    yPath += "," + to_string(planets[path[i]].position.y);
  }
  return 0;
}

int main(int argc, char *argv[])
{
  // Define command line options
  int opt, index;
  static struct option long_options[] =
  {
      {"map", required_argument, 0, 'm'},           // Input map
      {"maxPaths", required_argument, 0, 's'},      // Max number of best paths to generate with Yen's algorithm
      {"pathFactor", required_argument, 0, 'g'},    // nodeCount * pathFactor = paths to generate, capped at maxPaths
      {"test", required_argument, 0, 't'},          // Test mode (plots graph with Matplotlib, raw tests specific path)
      {"distanceScale", required_argument, 0, 'c'}, // Scales all planet distances by constant factor distanceScale
      {0, 0, 0, 0}
  };
  while ((opt = getopt_long(argc, argv, "m:", long_options, &index)) != -1)
  {
    switch (opt)
    {
    case 'm':
      fileName = optarg;
      break;
    case 's':
      maxPaths = atoi(optarg);
      break;
    case 'g':
      pathFactor = atoi(optarg);
      break;
    case 't':
      testMode = atoi(optarg);
      break;
    case 'c':
      distanceScale = atof(optarg);
      break;
    default:
      break;
    }
  }

  cout << endl
       << "Welcome Mission Control!" << endl;
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

  // Input and setup
  int error;
  do
  {
    planets.clear();
    error = parseCSV();
    fileName = "";
  } while (error != 0);

  // Pathfinding and speed/resource optimization
  initializeLists();
  int pathCount = pathFactor * nodeCount; // Number of best paths for Yen's algorithm to generate
  if (pathCount > maxPaths)
    pathCount = maxPaths;
  else if (pathCount < 50)
    pathCount = 50;
  vector<vector<int>> bestPaths;
  vector<int> bestPath;
  bool validPath = true;
  if (yens(bestPaths, pathCount) == -1) // Fills bestPaths with pathCount approximately best paths
  {
    cout << "Mission failed: Valid path could not be found. Please try a different map." << endl;
    validPath = false;
  }
  else // Optimizes speed for each path in bestPaths and selects the best out of them for output
  {
    double bestScore = INF;
    double score;
    vector<double> bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood;
    for (auto &path : bestPaths)
    {
      int pathLength = path.size();
      if (pathLength < 2)
        continue;
      vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength), rawOxygen(pathLength), rawFood(pathLength);
      fuel[0] = MAX_FUEL, oxygen[0] = MAX_OXYGEN, food[0] = MAX_FOOD;
      score = INF;
      error = optimizePath(path, score, speeds, fuel, oxygen, food, rawOxygen, rawFood);
      if (error == 0 && score < bestScore)
      {
        bestScore = score;
        bestPath = path;
        bestSpeeds = speeds;
        bestFuel = fuel;
        bestOxygen = oxygen;
        bestFood = food;
        bestRawOxygen = rawOxygen;
        bestRawFood = rawFood;
      }
    }
    if (bestScore < INF)
      printData(bestPath, bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood);
    else
    {
      cout << "Mission failed: Valid path could not be found. Please try a different map." << endl;
      validPath = false;
    }
  }
  long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
  double milliseconds = calcTime / 1000.0;
  cout << endl
       << "Calculation time: " << fixed << setprecision(6) << milliseconds / 1000 << " seconds (" << setprecision(3) << milliseconds << " milliseconds)" << endl
       << endl;

  // Testing
  if (testMode > 0)
  {
    string xEndpt = "", yEndpt = "", xNone = "", yNone = "", xFood = "", yFood = "", xOxygen = "", yOxygen = "", xHostile = "", yHostile = "", rHostile = "", xPath = "", yPath = "";
    for (int i = 0; i < nodeCount; i++)
    {
      PlanetData planet = planets[i];
      if (i == 0 || i == lastNodeIndex)
      {
        xEndpt += "," + to_string(planet.position.x);
        yEndpt += "," + to_string(planet.position.y);
        continue;
      }
      if (planet.hostileRadius > 0)
      {
        xHostile += "," + to_string(planet.position.x);
        yHostile += "," + to_string(planet.position.y);
        rHostile += "," + to_string(planet.hostileRadius);
        continue;
      }
      switch (planet.resupply)
      {
      case NONE:
        xNone += "," + to_string(planet.position.x);
        yNone += "," + to_string(planet.position.y);
        break;
      case FOOD:
        xFood += "," + to_string(planet.position.x);
        yFood += "," + to_string(planet.position.y);
        break;
      case OXYGEN:
        xOxygen += "," + to_string(planet.position.x);
        yOxygen += "," + to_string(planet.position.y);
        break;
      default:
        break;
      }
    }
    if (validPath)
    {
      for (int i = 0; i < bestPath.size(); i++)
      {
        xPath += "," + to_string(planets[bestPath[i]].position.x);
        yPath += "," + to_string(planets[bestPath[i]].position.y);
      }
    }
    ofstream outFile("PlotData.csv");
    if (outFile.is_open())
    {
      outFile << xEndpt.substr(1) << endl;
      outFile << yEndpt.substr(1) << endl;
      if (!xNone.empty())
        outFile << xNone.substr(1) << endl;
      if (!yNone.empty())
        outFile << yNone.substr(1) << endl;
      if (!xOxygen.empty())
        outFile << xOxygen.substr(1) << endl;
      if (!yOxygen.empty())
        outFile << yOxygen.substr(1) << endl;
      if (!xFood.empty())
        outFile << xFood.substr(1) << endl;
      if (!yFood.empty())
        outFile << yFood.substr(1) << endl;
      if (!xHostile.empty())
        outFile << xHostile.substr(1) << endl;
      if (!yHostile.empty())
        outFile << yHostile.substr(1) << endl;
      if (!rHostile.empty())
        outFile << rHostile.substr(1) << endl;
      if (validPath)
      {
        outFile << xPath.substr(1) << endl;
        outFile << yPath.substr(1) << endl;
      }
      outFile.close();
    }
    while (testMode == 1)
    {
      xPath = to_string(planets[0].position.x), yPath = to_string(planets[0].position.y);
      if (rawTestPath(xPath, yPath) == -1)
        break;
      ofstream outFile("PlotData.csv", ios::app);
      if (outFile.is_open())
      {
        outFile << xPath << endl;
        outFile << yPath << endl;
        outFile.close();
      }
    }
    while (testMode == 2)
    {
      if (testPath() == -1)
        break;
    }
  }
}