#pragma region README
/*
 * TSA Coding 2025-2026: Exploration Algorithm
 * TEAM: 2098-901
 * LANGUAGE: C++20 (chosen for performance), Python was also used but only for testing
 *
 * RESOURCES
 * The following classical algorithms were implemented in our program:
 * 1. Bidirectional A* (for pathfinding)
 * 2. Yen's algorithm (for pathfinding)
 * 3. Bisection method (for root-finding)
 * 4. Brent's method (for minimization)
 * As per the TSA event guide rules, only the language's standard library (std) was used (no 3rd-party libraries)
 * (Obviously no AI was used in the making of this project)
 *
 * NOTE: Since this program is kind of long, our team organized it into regions and added descriptive comments to improve readability
 */
#pragma endregion

#pragma region STD HEADERS
#include <iostream>
#include <fstream>
#include <iomanip>
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
#include <random>
#include <cstdlib>
using namespace std;
#pragma endregion

#pragma region GLOBAL VARIABLES

#define Kf 0.15
#define Kl 0.3
#define MaxFuel 3000
#define MaxOxygen 2000
#define MaxFood 2000
#define BUFFER 1e-12
#define INF numeric_limits<double>::infinity()
#define EPS std::numeric_limits<double>::epsilon()

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
struct HostilePlanet
{
  Position position;
  double radius;
  HostilePlanet(Position &p, double r)
  {
    position = p;
    radius = r;
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
  //Override comparison operator for Yen's algorithm set
  bool operator<(const Path &other) const
  {
    if (fabs(cost - other.cost) < EPS)
      return path < other.path; //Allow different paths with same cost
    return cost < other.cost;   //Sort by cost in Yen's algorithm set
  }
};

//Represents constraint boundary curve in speed optimization
struct Boundary
{
  double xb, yb;
  Boundary(double x, double y)
  {
    xb = x;
    yb = y;
  }
};

struct SlopeOptimizeData
{
  double fuel0, mvx, mvy;
  vector<Boundary> bs;
  double v1, v2;
  int error;
  SlopeOptimizeData(double f, double x, double y, vector<Boundary> b)
  {
    fuel0 = f;
    mvx = x;
    mvy = y;
    bs = b;
    v1 = 0.0;
    v2 = 0.0;
    error = 0;
  }
};

//Command line options (used for testing/debugging)
int batchSize = 0;
double distScale = 1.0;
int inputPath = 0;
int plotMap = 0;
int randomMap = 0;
int quickTest = 0;

//Random map generation (testing)
int lastPN;
double lastMP, lastHR, lastHP, lastOP, lastFP;

//Contains list of adjacent nodes for each node, speeds up neighbor expansion in pathfinding
vector<vector<int>> neighbors;
//3 adjacency matrix structures used to efficiently represent graph
vector<vector<double>> nodeCosts, nodeDistances, initialCosts;
//initialCosts used to restore changes to nodeCosts in Yen's algorithm

//Min cost per distance * euclidian distance to search target (sink for F and source for B) for admissible + consistent heuristic
vector<double> hF, hB;
vector<PlanetData> planets;
vector<HostilePlanet> hostilePlanets;
int nodeCount, foodCount, oxygenCount, lastNodeIndex, longestNameLength;
chrono::steady_clock::time_point startTime;
string fileName = "", mapName;

const double phi = (3 - sqrt(5)) / 2; //Golden section ratio for Brent's minimization
//Speed optimization by first derivative test: cost = c(x) = time + fuel + food + oxygen = d/v + Kf*d*v^2 + Kl*d/v + Kl*d/v
//Cost minimized at c'(x) = 0 => (-d + 2*Kf*d*v^3 - Kl*d - Kl*d)/v^2 = 0 => v = cbrt((1 + Kl+Kl) / (2*Kf))
const double idealSpeed = cbrt((1 + 2 * Kl) / (2 * Kf));
//One resupply type remaining, ideal speed can disregard consumption of 1 resource type (1 Kl term)
const double singleResupplySpeed = cbrt((1 + Kl) / (2 * Kf));
//Both resupply types remaining, ideal speed can disregard consumption of both Kl terms
const double doubleResupplySpeed = 1 / cbrt(2 * Kf);

#pragma endregion

#pragma region INPUT

string toLower(const string &str)
{
  string str2 = str;
  for (char &c : str2)
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

double calculateDistance(const Position &pos1, const Position &pos2)
{
  double dx = pos2.x - pos1.x, dy = pos2.y - pos1.y;
  return sqrt(dx * dx + dy * dy);
}

//Extracts data from inputted CSV map with error handling
int parseCSV()
{
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

  //Start computation time immediately after user enters file name.
  startTime = chrono::steady_clock::now();

  do
  {
    getline(iFile, line);
  } while (trim(line).empty()); //Skip empty lines
  if (count(line.begin(), line.end(), ',') != 5)
  {
    cout << "CSV Error: Entry 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
    return -1;
  }
  int commaIndex;
  string headerSections[] = {"planet name", "x", "y", "type", "resupply", "hostile_radius"};
  for (int i = 0; i < 5; i++)
  {
    commaIndex = line.find(',');
    if (trim(toLower(line.substr(0, commaIndex))) != headerSections[i])
    {
      cout << "CSV Error: Entry 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);
  }

  string planetName, type, resupply, substring;
  vector<int> hostileIndices;
  int lineNumber = 1;
  double x, y, hostileRadius;
  while (getline(iFile, line))
  {
    if (trim(line).empty())
      continue; //Skip empty lines
    lineNumber++;
    int commas = count(line.begin(), line.end(), ',');
    if (commas != 5)
    {
      cout << "CSV Error: Expected 5 commas in entry " << lineNumber << " (found " << commas << ")." << endl;
      return -1;
    }

    commaIndex = line.find(',');
    planetName = trim(line.substr(0, commaIndex));

    bool validName, alphanumeric = false;
    for (const char &c : planetName)
    {
      if (isalnum(c))
      {
        alphanumeric = true;
        continue;
      }
      switch (c)
      {
      //Spaces, dashes, and parentheses allowed
      case ' ':
      case '-':
      case '(':
      case ')':
        continue;
      default:
        validName = false;
        break;
      }
      if (!validName)
        break;
    }
    //Planet names can't be all special characters
    validName = alphanumeric;

    if (planetName.empty() || !validName)
    {
      cout << "CSV Error: Invalid planet name \"" << planetName << "\" found on entry " << lineNumber << ". Planet names must be alphanumeric (spaces, dashes, and parentheses accepted)." << endl;
      return -1;
    }
    int planetCount = planets.size();
    for (int i = 0; i < planetCount; i++)
    {
      if (planets[i].name == planetName)
      {
        cout << "CSV Error: Duplicate planet name \"" << planetName << "\" found on entries " << i + 2 << " and " << lineNumber << "." << endl;
        return -1;
      }
    }
    line = line.substr(commaIndex + 1);

    commaIndex = line.find(',');
    substring = trim(line.substr(0, commaIndex));
    try
    {
      x = stod(substring);
    }
    catch (...)
    {
      cout << "CSV Error: The x-coordinate (\"" << substring << "\") of planet " << planetName << " (entry " << lineNumber << ") must be numerical." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);

    commaIndex = line.find(',');
    substring = trim(line.substr(0, commaIndex));
    try
    {
      y = stod(substring);
    }
    catch (...)
    {
      cout << "CSV Error: The y-coordinate (\"" << substring << "\") of planet " << planetName << " (entry " << lineNumber << ") must be numerical." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);

    commaIndex = line.find(',');
    type = trim(toLower(line.substr(0, commaIndex)));
    if (type != "neutral" && type != "hostile")
    {
      cout << "CSV Error: The type (\"" << type << "\") of planet " << planetName << " (entry " << lineNumber << ") must be \"neutral\" or \"hostile\"." << endl;
      return -1;
    }
    line = line.substr(commaIndex + 1);

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
      cout << "CSV Error: The resupply (\"" << resupply << "\") of planet " << planetName << " (entry " << lineNumber << ") must be \"\" (none), \"food\", or \"oxygen\"." << endl;
      return -1;
    }
    line = trim(line.substr(commaIndex + 1));

    try
    {
      hostileRadius = stod(line);
    }
    catch (...)
    {
      cout << "CSV Error: The hostile radius (\"" << line << "\") of planet " << planetName << " (entry " << lineNumber << ") must be numerical." << endl;
      return -1;
    }
    if (type == "neutral" && hostileRadius != 0)
    {
      cout << "CSV Error: The hostile radius (" << hostileRadius << ") of neutral planet " << planetName << " (entry " << lineNumber << ") must be 0." << endl;
      return -1;
    }
    else if (type == "hostile")
    {
      if (lineNumber == 2)
      {
        cout << "CSV Error: The first planet (" << planetName << ") cannot be hostile." << endl;
        return -1;
      }
      else if (hostileRadius <= 0)
      {
        cout << "CSV Error: The hostile radius (" << hostileRadius << ") of hostile planet " << planetName << " (entry " << lineNumber << ") must be positive." << endl;
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
    cout << "CSV Error: There must be at least 2 planets in the map." << endl;
    return -1;
  }
  if (planets.back().hostileRadius != 0)
  {
    cout << "CSV Error: The target planet (" << planets.back().name << ") cannot be hostile." << endl;
    return -1;
  }
  nodeCount = lineNumber - 1, lastNodeIndex = lineNumber - 2;

  //Remove unreachable planets
  set<int, greater<int>> unreachableIndices;
  int hostileCount = hostileIndices.size();
  for (int i : hostileIndices)
  {
    PlanetData &planet = planets[i];
    Position &pos = planet.position;
    double hostileRadius = planet.hostileRadius;
    for (int j = 0; j < nodeCount; j++)
    {
      if (i == j)
        continue;
      PlanetData &planet2 = planets[j];
      double d = calculateDistance(pos, planet2.position), r = planet2.hostileRadius;
      //Unreachable if neutral and lying within hostile radius or hostile and fully enclosed (including radius) in other hostile radius
      if (r <= 0 && d <= hostileRadius)
      {
        unreachableIndices.insert(j);
        if (j == 0)
        {
          cout << "CSV Error: The first planet (" << planets[0].name << ") is in the hostile radius of planet " << planet.name << "." << endl;
          return -1;
        }
        else if (j == lastNodeIndex)
        {
          cout << "CSV Error: The target planet (" << planets.back().name << ") is in the hostile radius of planet " << planet.name << "." << endl;
          return -1;
        }
      }
      else if (r > 0 && r <= hostileRadius - d)
      {
        pair<set<int>::iterator, bool> result = unreachableIndices.insert(j);
        if (result.second)
          hostileCount--;
      }
    }
  }
  for (int i : unreachableIndices)
    planets.erase(planets.begin() + i);

  nodeCount = planets.size();
  lastNodeIndex = nodeCount - 1;
  if (fabs(distScale - 1) > EPS) //Scale map (used to test extreme conditions)
  {
    for (PlanetData &planet : planets)
    {
      Position &pos = planet.position;
      pos.x *= distScale;
      pos.y *= distScale;
      planet.hostileRadius *= distScale;
    }
  }
  hostilePlanets.reserve(hostileCount);
  foodCount = 0;
  oxygenCount = 0;
  for (PlanetData &planet : planets)
  {
    if (planet.hostileRadius > 0)
      hostilePlanets.emplace_back(planet.position, planet.hostileRadius);
    switch (planet.resupply)
    {
    case OXYGEN:
      oxygenCount++;
      break;
    case FOOD:
      foodCount++;
      break;
    default:
      break;
    }
  }
  iFile.close();
  return 0;
}

#pragma endregion

#pragma region PATHFINDING

//Returns cost (time + resource consumption) of edge connecting two nodes
double calculateCost(const int node1, const int node2)
{
  double d = nodeDistances[node1][node2];
  //Determine speed based on probability that there is food/oxygen resupply left in path
  double foodProb = foodCount / (double)nodeCount;
  double oxygenProb = oxygenCount / (double)nodeCount;
  double fuelConsumed, oxygenConsumed, foodConsumed, t, speed;
  switch (planets[node2].resupply)
  {
  case OXYGEN:
    speed = doubleResupplySpeed * cbrt(1 + (1 + foodProb) * Kl), t = d / speed;
    oxygenConsumed = 0;
    foodConsumed = (1 - foodProb) * Kl * t;
    break;
  case FOOD:
    speed = doubleResupplySpeed * cbrt(1 + (1 + oxygenProb) * Kl), t = d / speed;
    foodConsumed = 0;
    oxygenConsumed = (1 - oxygenProb) * Kl * t;
    break;
  default:
    speed = doubleResupplySpeed * cbrt(1 + (2 + oxygenProb + foodProb) * Kl), t = d / speed;
    oxygenConsumed = (1 - oxygenProb) * Kl * t;
    foodConsumed = (1 - foodProb) * Kl * t;
    break;
  }
  fuelConsumed = Kf * d * speed * speed;
  return t + fuelConsumed + oxygenConsumed + foodConsumed;
}

bool edgeInHostileRadius(const int startNode, const int endNode)
{
  //Calculates intersection of line through (x1,y1) and (y1,y2) with circle of radius r at (x3,y3)
  Position &start = planets[startNode].position, end = planets[endNode].position;
  //Line eq. (parametric): x = x1 + cos(theta)*d, y = y1 + sin(theta)*d where d = distance along line and theta = atan(slope)
  double x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y, rise = y2 - y1, run = x2 - x1;
  double theta = atan2(rise, run), cosTheta = cos(theta), sinTheta = sin(theta); //Precompute trig for performance
  double l = run * run + rise * rise; //Length of line
  for (HostilePlanet &planet : hostilePlanets)
  {
    Position &center = planet.position;
    //Circle eq.: (x-x3)^2 + (y-y3)^2 = r^2
    double x3 = center.x, y3 = center.y, r = planet.radius + BUFFER, dx = x1 - x3, dy = y1 - y3;
    //Plugging line eqs. into circle eq. and applying trig identities yields quadratic a*d^2 + b*d + c = 0
    double b = 2 * dx * cosTheta + 2 * dy * sinTheta, c = dx * dx + dy * dy - r * r; //a=1
    double discriminant = b * b - 4 * c;
    //If discriminant < 0, no solution and edge does not cross hostile circle
    if (discriminant < 0)
      continue;
    double d1 = (-b + sqrt(discriminant)) / 2, d2 = (-b - sqrt(discriminant)) / 2; //Roots from quadratic formula
    //If both values for d are lower or higher than edge length, hostile circle is still not crossed
    if ((d1 < 0 && d2 < 0) || (d1 * d1 > l && d2 * d2 > l))
      continue;
    return true;
  }
  return false;
}

//Provides pathfinding setup
void initializeLists()
{
  Position sourcePos = planets[0].position, sinkPos = planets[lastNodeIndex].position;
  neighbors = vector<vector<int>>(nodeCount);
  nodeCosts = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  nodeDistances = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  initialCosts = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
  //Heuristic constant for A* pathfinding (minimum cost per unit distance traveled)
  const double heuristic = (1 + Kl) / singleResupplySpeed + Kf * singleResupplySpeed * singleResupplySpeed;
  hF = vector<double>(nodeCount, 0), hB = vector<double>(nodeCount, 0);

  int maxNeighbors = nodeCount - 1 - hostilePlanets.size();
  for (vector<int> &neighborList : neighbors)
    neighborList.reserve(maxNeighbors);
  for (int i = 0; i < nodeCount; i++)
  {
    double hostileRadius = planets[i].hostileRadius;
    for (int j = i + 1; j < nodeCount; j++)
    {
      //Eliminate edges crossing hostile radius
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

      //A* heuristic for each node
      if (j == lastNodeIndex)
        hF[i] = heuristic * distance;
      if (i == 0)
        hB[j] = heuristic * distance;
    }
    nodeCosts[i][i] = initialCosts[i][i] = INF;
  }
}

//Runs forward + backward A* search until they meet to form best path from Earth to target
int bidirectionAStar(vector<int> &bestPath, const int earth, const int target)
{
  //Forward and backward search priority queues of (cost, index) sorting most promising (least cost) node at top
  priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pqF, pqB;
  pqF.emplace(0, earth);
  pqB.emplace(0, target);
  vector<double> gF(nodeCount, INF), gB(nodeCount, INF); //Forward and backward g-scores/cumulative costs
  gF[earth] = gB[target] = 0;                                           
  vector<bool> visitedF(nodeCount, false), visitedB(nodeCount, false); 
  vector<int> parentsF(nodeCount, -1), parentsB(nodeCount, -1);        

  double bestCost = INF;
  int meetingNode = -1;
  while (!pqF.empty() && !pqB.empty())
  {
    int node1F = pqF.top().second, node1B = pqB.top().second;
    if (bestCost < INF && gF[node1F] + gB[node1B] >= bestCost)
        break;
    
    //Forward search
    pqF.pop();
    if (!visitedF[node1F])
    {
      visitedF[node1F] = true;
      double g1 = gF[node1F];
      vector<int> &nodeNeighbors = neighbors[node1F];
      for (int node2 : nodeNeighbors)
      {
        double cost = nodeCosts[node1F][node2];
        if (cost == INF || visitedF[node2])
          continue;
        double newG = g1 + cost;
        if (newG < gF[node2])
        {
          //Relax edge
          gF[node2] = newG;
          parentsF[node2] = node1F;
          pqF.emplace(newG + hF[node2], node2);
          //Update best path if both searches have visited this node (potential meeting point)
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
    
    //Backward search
    pqB.pop();
    if (!visitedB[node1B])
    {
      visitedB[node1B] = true;
      double g1 = gB[node1B];
      vector<int> &nodeNeighbors = neighbors[node1B];
      for (int node2 : nodeNeighbors) //Expand adjacent nodes
      {
        double cost = nodeCosts[node2][node1B];
        if (cost == INF || visitedB[node2])
          continue;
        double newG = g1 + cost;
        if (newG < gB[node2])
        {
          //Relax edge
          gB[node2] = newG;
          parentsB[node2] = node1B;
          pqB.emplace(newG + hB[node2], node2);
          //Update best path if both searches have visited this node (potential meeting point)
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

  //Reconstruct path
  if (meetingNode == -1)
    return -1;
  vector<int> leftPath, rightPath;
  int node = meetingNode;
  while (node != -1)
  {
    //Build left path before meeting node
    leftPath.push_back(node);
    if (node == earth)
      break;
    node = parentsF[node];
  }
  if (leftPath.back() != earth)
    return -1; //Path didn't connect to earth
  reverse(leftPath.begin(), leftPath.end());
  node = meetingNode;
  while (node != -1 && node != target)
  {
    //Build right path after meeting node
    node = parentsB[node];
    if (node != -1)
      rightPath.push_back(node);
  }
  if (meetingNode != target && (rightPath.empty() || rightPath.back() != target))
    return -1; //Path didn't connect to target
  bestPath = leftPath;
  bestPath.insert(bestPath.end(), rightPath.begin(), rightPath.end());
  return bestPath.empty() ? -1 : 0;
}

//Finds pathCount best paths, repeatedly calls bidirectional A* after modifying graph (blocking root path of previous best paths)
int yens(vector<vector<int>> &bestPaths, int pathCount)
{
  vector<int> bestPath, sameRootPaths, rootPath, spurPath, totalPath;
  int initialSize = bestPaths.size();
  bestPaths.reserve(initialSize + pathCount);
  int k0 = 0;
  if (initialSize == 0) //Called on first batch (see main())
  {
    //Find best path from source (Earth) to sink (Target) for first batch
    if (bidirectionAStar(bestPath, 0, lastNodeIndex) == -1)
      return -1;
    bestPaths.push_back(bestPath);
    k0 = 1; //Find pathCount-1 other paths since 1 already found
  }
  set<Path> potentialPaths; //Use set to avoid duplicates and automatic sorting
  for (int k = k0; k < pathCount; k++)
  {
    vector<int> &lastBestPath = bestPaths.back();
    int n = lastBestPath.size() - 1;
    for (int i = 0; i < n; i++)
    {
      int bestPathsCount = bestPaths.size();
      rootPath.assign(lastBestPath.begin(), lastBestPath.begin() + i + 1); //rootPath = sequence of nodes from source to spur node
      sameRootPaths.clear();
      sameRootPaths.reserve(bestPathsCount);
      //Modify graph to produce different best path
      for (int j = 0; j < bestPathsCount; j++)
      {
        vector<int> &path = bestPaths[j];
        //Remove edges from spur nodes of other paths sharing same same root to prevent duplicate paths
        if (rootPath.size() <= path.size() && equal(rootPath.begin(), rootPath.end(), path.begin()))
        {
          sameRootPaths.push_back(j);
          nodeCosts[path[i]][path[i + 1]] = INF;
        }
      }
      for (int j = 0; j < i; j++)
      {
        //Remove all nodes in root path from graph to prevent cycles
        int node = lastBestPath[j];
        for (int m = 0; m < nodeCount; m++)
          nodeCosts[node][m] = nodeCosts[m][node] = INF;
      }
      int spurNode = lastBestPath[i]; //Retrieve spur node from previous shortest path
      spurPath.clear();
      int error = bidirectionAStar(spurPath, spurNode, lastNodeIndex); //Calculate spur path from spur node to sink

      //Restore removed edges/nodes
      int sameRootCount = sameRootPaths.size();
      for (int j = 0; j < sameRootCount; j++)
      {
        vector<int> &path = bestPaths[sameRootPaths[j]];
        int node1 = path[i], node2 = path[i + 1];
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
      totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end()); //Combine root and spur path into complete path
      int pathLength = totalPath.size();
      double cost = 0;
      for (int j = 1; j < pathLength; j++)
        cost += initialCosts[totalPath[j - 1]][totalPath[j]];
      potentialPaths.emplace(totalPath, cost); //Adds complete path sorted by ascending cost, avoiding duplicates (set)
    }

    if (potentialPaths.empty()) //No spur paths (e.g. all already exhausted or none exist)
      break;
    auto it = potentialPaths.begin();
    const Path &path = *it;
    bestPaths.push_back(path.path);
    potentialPaths.erase(it);
  }
  if (bestPaths.size() == initialSize)
    return -1; //Did not find any new best paths
  return 0;
}

#pragma endregion

#pragma region SPEED/RESOURCE OPTIMIZATION

//Bisection algorithm to find root c of func in [a, b], works by repeatedly halving interval until func(c) is approximately 0
double bisection(double a, double b, void *args, double (*func)(double, void *), const double tolerance = 1e-6)
{
  double c;
  c = 0.5 * (a + b);
  double fa = func(a, args);
  double fb = func(b, args);
  while (fabs(b - a) > tolerance * c)
  {
    double fc = func(c, args);
    if (fabs(fc) < BUFFER)
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

//Equations to find intersection of boundary curves
double func(double theta, void *args)
{
  Boundary *fa = (Boundary *)args;
  double cosTheta = cos(theta), sinTheta = sin(theta);
  return fa->xb * sinTheta + fa->yb * cosTheta - sinTheta * cosTheta;
}

double derivFunc(double theta, void *args)
{
  Boundary *fa = (Boundary *)args;
  double cosTheta = cos(theta), sinTheta = sin(theta);
  return fa->xb * cosTheta - fa->yb * sinTheta - (cosTheta * cosTheta - sinTheta * sinTheta);
}

//Optimizes speed solution along line of slope m (see optimizeSpeed())
int getIdealSpeed(double &v1, double &v2, double &score, const double m,
                  const double b, const double c, const vector<Boundary> &bs)
{
  double outerIntersection = 1 / sqrt(1 + m * m); //Intersection of line and circle (fuel constraint)
  double innerIntersection = 0.0;
  double xi, x1, y1;
  for (const Boundary &boundary : bs) //Intersection of line and food/oxygen boundary curves
  {
    x1 = boundary.xb;
    y1 = boundary.yb;
    if (fabs(x1) < EPS)
      xi = y1 / m;
    else if (fabs(y1) < EPS)
      xi = x1;
    else
      xi = x1 + y1 / m;
    if (xi > innerIntersection)
      innerIntersection = xi;
  }
  if (innerIntersection >= outerIntersection)
    return -1;
  double v1Ideal = cbrt(0.5 * b / c); //Analytic solution of ideal speed on line
  double di = BUFFER * (outerIntersection - innerIntersection);
  double v1Min = innerIntersection + di, v1Max = outerIntersection - di;
  int returnVal;
  //Clamps solution to be in the area bounded by constraint curves
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

double slopeOptimizeFunc(double x, void *arg)
{
  SlopeOptimizeData *a = (SlopeOptimizeData *)arg;
  double s = tan(x);
  double b = a->mvx + a->mvy / s;
  double c = a->fuel0 * (1 + s * s);
  double r;
  a->error = getIdealSpeed(a->v1, a->v2, r, s, b, c, a->bs);
  if (a->error != 0)
    r = INF;
  return r;
}

//Uses Brent's minimization method to find best speeds (see optimizeSpeed())
int minimize(double &xm, double &fm, double a, double b,
             void *arg, double (*func)(double, void *), const double tolerance = 1e-8, const int maxIter = 500)
{
  //Initializes bracketing points
  int returnVal;
  double x = a + phi * (b - a);
  double fx = func(x, arg);
  double w = x, fw = fx; //w = previous best point
  double v = x, fv = fx; //v = previous of previous best point
  double d = 0.0;
  double e = 0.0;

  for (int iter = 0; iter < maxIter; iter++)
  {
    double midpoint = 0.5 * (a + b);
    double tol1 = tolerance * fabs(x);
    double tol2 = 2.0 * tol1;

    //Stops when interval sufficiently small
    if (fabs(x - midpoint) <= (tol2 - 0.5 * (b - a)))
    {
      xm = x;
      fm = fx;
      return 0;
    }

    //Try parabolic interpolation
    if (fabs(e) > tol1)
    {
      //Fits parabola through (w, fw), (x, fx), (v, fv)
      double r = (x - w) * (fx - fv);
      double q = (x - v) * (fx - fw);
      double p = (x - v) * q - (x - w) * r;
      q = 2.0 * (q - r);
      q > 0.0 ? p = -p : q = -q;

      if (fabs(p) < fabs(0.5 * q * e) && p > q * (a - x) && p < q * (b - x))
      {
        d = p / q;
        double u = x + d;

        //f cannot be evaluated too close to a or b
        if ((u - a) < tol2 || (b - u) < tol2)
          d = (x < midpoint) ? tol1 : -tol1;
      }
      else
      {
        e = (x >= midpoint) ? (a - x) : (b - x);
        d = phi * e;
      }
    }
    else
    {
      //Fall back to Golden section search if parabolic step unacceptable
      e = (x >= midpoint) ? (a - x) : (b - x);
      d = phi * e;
    }

    //Evaluates function at new point
    double u = (fabs(d) >= tol1) ? (x + d) : (x + ((d > 0.0) ? tol1 : -tol1));
    double fu = func(u, arg);
    //Updates bracketing triplet
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

//Divides path into two segments: i0-1 to im and im to i1 (i0=im for our calls) and optimizes speed for i0-1 to im
//Optimization is done by scaling ideal speeds for nodes in i0-1 to im by constant factor and for nodes in im to i1 by constant factor
//Method 3 in OptimizePath() uses a loop to set each node to im and i0, thus eventually optimizing the entire path
int optimizeSpeed(const int i0, const int im, const int i1, const vector<int> &path, const vector<double> &mb,
                  vector<double> &speeds, const vector<double> &fuel, const vector<double> &oxygen, const vector<double> &food)
{
  //Each constraint (fuel, food, oxygen) can be represented by a boundary curve on a graph that depends on the resupply type of each node
  //The area inside these boundaries is the solution space that will satisfy these constraints (resources never drop below 0)
  int returnVal = -1;
  double v1, v2, score;
  double fuel0 = fuel[i0 - 1], oxygen0 = oxygen[i0 - 1], food0 = food[i0 - 1];
  int ni = i1 - i0 + 2;
  vector<double> cs(ni), ci(ni);
  cs[0] = ci[0] = 0.0;
  double dx = 0.0, dy = 0.0, sx = 0.0, sy = 0.0, mx = 0.0, my = 0.0;
  int i;
  for (i = i0; i <= i1; i++) //Calculates values and cumulative sums used later to determine boundaries
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
  //The fuel constraint introduces the boundary (v1)^2 + (v2)^2 < 1
  //Graphically this is the area bounded by a circle of radius 1 (and in the first quadrant because all speeds and scale factors are positive)
  double vx = sqrt(Kf / fuel0 * sx); //Constant factor for i0-1 to m is v1/vx
  double vy = sqrt(Kf / fuel0 * sy); //Constant factor for m to i1 is v2/vy
  vector<Boundary> bs;
  for (int ir = 0; ir < 2; ir++) //Fills bs with list of food/oxygen boundary curves introduced by each resupply node
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
        double bx = vx * Kl / q0, by = vy * Kl / q0;
        if (i <= im)
        {
          ix = ci[i - i0 + 1] - ci[bestIndex - i0 + 1];
          bs.emplace_back(bx * ix, 0.0); //Vertical boundary curve
        }
        else if (bestIndex < im)
        {
          ix = ci[im - i0 + 1] - ci[bestIndex - i0 + 1];
          iy = ci[i - i0 + 1] - ci[im - i0 + 1];
          bs.emplace_back(bx * ix, by * iy); //Curvilinear boundary
        }
        else
        {
          iy = ci[i - i0 + 1] - ci[bestIndex - i0 + 1];
          bs.emplace_back(0, by * iy); //Horizontal boundary curve
        }
        bestIndex = i;
        q0 = ir == 0 ? MaxOxygen : MaxFood;
      }
    }
  }

  double thetaMin = 0.0, thetaMax = M_PI_2; //Angles of lines from origin to intersection points of curves (used to find bounded area)
  for (i = 0; i < bs.size(); i++)
  {
    if (fabs(bs[i].xb) < EPS)
    {
      bs[i].yb >= 1 ? thetaMin = M_PI_2 : thetaMin = max(thetaMin, asin(bs[i].yb));
    }
    else if (fabs(bs[i].yb) < EPS)
    {
      bs[i].xb >= 1 ? thetaMax = 0.0 : thetaMax = min(thetaMax, acos(bs[i].xb));
    }
    else
    {
      //Determines the intersection point of curves using root-finding algorithm (bisection)
      //Find minimum using 1st Derivative Test to determine interval of root (required for bisection)
      double tm = bisection(0, M_PI_2, &bs[i], derivFunc);
      thetaMin = max(thetaMin, bisection(0, tm, &bs[i], func));
      thetaMax = min(thetaMax, bisection(tm, M_PI_2, &bs[i], func));
      //Tightest thetaMin/thetaMax is used (to satisfy every constraint)
    }
    if (thetaMin >= thetaMax)
      return -1;
  }
  double a, c, s;
  double ds = (thetaMax - thetaMin) * BUFFER;
  thetaMin += ds, thetaMax -= ds;                                   //Slightly narrow boundaries for affordance
  double idealSlope = sqrt(sy / sx), idealTheta = atan(idealSlope); //Ideal theta defined when scale factors are 1
  if (idealTheta < thetaMin)
  {
    idealTheta = thetaMin; //Clamp idealTheta to thetaMin and thetaMax
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

  //Iterates through lines with angles from thetaMin to thetaMax, finds best speed for point on each line within solution space
  //Minimizes score of best speeds using Brent's method to find absolute best
  double mvx = mx * vx;
  double mvy = my * vy;
  double sb = INF;
  double v1b = 0.0, v2b = 0.0;
  SlopeOptimizeData arg(fuel0, mvx, mvy, bs);
  minimize(s, sb, thetaMin, thetaMax, &arg, slopeOptimizeFunc);
  v1b = arg.v1;
  v2b = arg.v2;

  //Test solution against ideal slope as a final check (needed if min could not be found)
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
    speeds[i] *= v1 / vx; //Scale speeds for nodes in segment i0-1 to im by constant factor
  for (i = im + 1; i <= i1; i++)
    speeds[i] *= v2 / vy; //Scale speeds for nodes in segment im to i1 by constant factor

  return returnVal;
}

double scoreFunc(vector<int> &resupplies, vector<double> &resupDists, vector<double> &resupSpeeds,
                 int lastOxygen, int lastFood,
                 vector<double> &jac, vector<double> &djac)
{
  int resupplyCount = resupplies.size();
  double score = 0.0;
  jac.assign(resupplyCount, 0.0);
  djac.assign(resupplyCount, 0.0);
  for (int i = 1; i < resupplyCount; i++)
  {
    int ip = abs(resupplies[i]);
    double distance = resupDists[i];
    double speed = resupSpeeds[i];
    double t = distance / speed;
    double tos = t / speed;
    double tos2 = 2 * tos / speed;
    score += t;
    jac[i] += -tos;
    djac[i] += tos2;
    score += Kf * distance * speed * speed;
    jac[i] += 2 * Kf * distance * speed;
    djac[i] += 2 * Kf * distance;
    if (ip > lastOxygen)
    {
      score += Kl * t;
      jac[i] -= tos;
      djac[i] += tos2;
    }
    if (ip > lastFood)
    {
      score += Kl * t;
      jac[i] -= tos;
      djac[i] += tos2;
    }
  }
  score -= MaxFuel + MaxFood + MaxOxygen;
  return score;
}

int calculateResources(const vector<int> &path, vector<double> &speeds, vector<double> &fuel, vector<double> &oxygen,
                       vector<double> &food, vector<double> &rawOxygen, vector<double> &rawFood, double &score)
{
  int pathLength = path.size();
  if (speeds.size() != pathLength)
  {
    speeds = vector<double>(pathLength, idealSpeed);
    fuel = vector<double>(pathLength);
    oxygen = vector<double>(pathLength);
    food = vector<double>(pathLength);
    rawOxygen = vector<double>(pathLength);
    rawFood = vector<double>(pathLength);
  }
  double fuelLeft = MaxFuel, oxygenLeft = MaxOxygen, foodLeft = MaxFood;
  score = 0.0;
  int error = 0;
  for (int i = 1; i < pathLength; i++)
  {
    int node = path[i];
    double speed = speeds[i];
    double d = nodeDistances[path[i - 1]][node];

    double t = d / speed;
    double fuelConsumed = Kf * d * speed * speed;
    double consumed = Kl * t; //Oxygen/food consumed
    score += t;
    fuelLeft -= fuelConsumed;
    oxygenLeft -= consumed;
    foodLeft -= consumed,

    rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
    if (fuelLeft < EPS || oxygenLeft < EPS || foodLeft < EPS)
      error = -1;
    switch (planets[node].resupply)
    {
    case OXYGEN:
      oxygenLeft = MaxOxygen;
      break;
    case FOOD:
      foodLeft = MaxFood;
      break;
    default:
      break;
    }
    fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
  }
  score -= fuelLeft + oxygenLeft + foodLeft;
  return error;
}

int optimizePath(const vector<int> &path, double &score, vector<double> &speeds, vector<double> &fuel,
                 vector<double> &oxygen, vector<double> &food, vector<double> &rawOxygen, vector<double> &rawFood)
{
  int error = 0;
  bool foundOxygenResupply = false, foundFoodResupply = false;
  int pathLength = path.size();
  vector<double> mb(pathLength); //Constant used for speed optimization, either 1, 1+Kl, or 1+2Kl
  int lastOxygen = 0, lastFood = 0;
  vector<int> resupplies;
  vector<double> resupDists; //Distance between subsequent resupplies

  resupplies.push_back(0);
  resupDists.push_back(0.0);
  double d = 0.0;
  //Construct list of resupply nodes
  for (int i = 1; i < pathLength; i++)
  {
    d += nodeDistances[path[i - 1]][path[i]];
    switch (planets[path[i]].resupply)
    {
    case OXYGEN:
      resupplies.push_back(i);
      resupDists.push_back(d);
      d = 0.0;
      lastOxygen = i;
      break;
    case FOOD:
      resupplies.push_back(-i);
      resupDists.push_back(d);
      d = 0.0;
      lastFood = i;
      break;
    default:
      break;
    }
  }
  resupplies.push_back(pathLength - 1);
  resupDists.push_back(d);

  int resupplyCount = resupplies.size();
  vector<double> resupSpeeds(resupplyCount);

  //Set ideal speed by disregarding both, one, or none of the Kl terms if node is before both, one or none of the last resupply types
  int minLastResupply, maxLastResupply;
  if (lastOxygen < lastFood)
  {
    minLastResupply = lastOxygen;
    maxLastResupply = lastFood;
  }
  else
  {
    minLastResupply = lastFood;
    maxLastResupply = lastOxygen;
  }
  for (int i = 1; i <= minLastResupply; i++)
  {
    mb[i] = 1.0;
    speeds[i] = doubleResupplySpeed;
  }
  for (int i = minLastResupply + 1; i <= maxLastResupply; i++)
  {
    mb[i] = 1 + Kl;
    speeds[i] = singleResupplySpeed;
  }
  for (int i = maxLastResupply + 1; i < pathLength; i++)
  {
    mb[i] = 1 + 2 * Kl;
    speeds[i] = idealSpeed;
  }

  error = calculateResources(path, speeds, fuel, oxygen, food, rawOxygen, rawFood, score);
  if (error == 0)
    return 0;

  //If ideal case fails, adjust all speeds by constant factor
  double totalFuelConsumed = MaxFuel - fuel.back();
  double speedFactor = sqrt(MaxFuel / totalFuelConsumed);
  double fuelLeft = MaxFuel, oxygenLeft = MaxOxygen, foodLeft = MaxFood;
  error = 0;
  if (totalFuelConsumed >= MaxFuel)
  {
    score = 0.0;
    for (int i = 1; i < pathLength; i++)
    {
      int node = path[i];
      speeds[i] *= speedFactor;
      double speed = speeds[i], distance = nodeDistances[path[i - 1]][node], t = distance / speed, consumed = Kl * t;
      fuelLeft -= Kf * distance * speed * speed, oxygenLeft -= consumed, foodLeft -= consumed;
      if (oxygenLeft < BUFFER || foodLeft < BUFFER || fuelLeft < BUFFER)
        error = -1;
      rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
      switch (planets[node].resupply)
      {
      case OXYGEN:
        oxygenLeft = MaxOxygen;
        break;
      case FOOD:
        foodLeft = MaxFood;
        break;
      default:
        break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      score += t;
    }
    score -= fuel.back() + oxygen.back() + food.back();
    if (error == 0)
      return 0;
  }

  //If constant factor fails, optimize each node's speeds using a combination of algorithmic, analytical, and graphical means
  //Each leg between subsequent resupplies will have the same speed
  //(it can be mathmatically proven that this is ideal, e.g. with Lagrange multipliers, Karush–Kuhn–Tucker conditions)
  for (int i = 1; i < resupplyCount; i++)
  {
    int ip = abs(resupplies[i]);
    resupSpeeds[i] = speeds[ip];
  }
  
  double prevScore = INF;
  vector<double> speeds0, fuel0, oxygen0, food0;
  double fuelLeft0, oxygenLeft0, foodLeft0;
  vector<double> jac(resupplyCount), djac(resupplyCount);
  double dsr = 1.2, dsc = 1.0, mja = 1e-5, mdv = 0.75, mds = 0.025;
  //Iterate until solution is self-consistent (speed converges), guarantees correctness despite approximations
  //Capped at 10 iterations to prevent infinite loop, always converges earlier in testing
  for (int iter = 0; iter < 10; iter++)
  {
    if (iter > 0)
    {
      speeds0 = speeds;
      fuel0 = fuel;
      oxygen0 = oxygen;
      food0 = food;
      fuelLeft0 = fuelLeft;
      oxygenLeft0 = oxygenLeft;
      foodLeft0 = foodLeft;
      score = scoreFunc(resupplies, resupDists, resupSpeeds, lastOxygen, lastFood, jac, djac);
      double ds = mds * fabs(score), dv = 0.0, jdv = 0.0;
      for (int i = 1; i < resupplyCount; i++)
      {
        jdv = 0.0;
        dv = 0.0;
        if (fabs(djac[i]) > mja)
          jdv = min(1.0, dsc) * jac[i] / djac[i];
        if (fabs(jac[i]) > mja)
          dv = dsc * ds / jac[i];
        if (dv * jdv > 1e-16)
        {
          if (fabs(dv) > fabs(jdv))
            dv = jdv;
          double adv = fabs(dv);
          double rv = mdv * speeds[i];
          if (adv > rv)
            dv *= rv / adv;
          resupSpeeds[i] -= dv;
          int im = abs(resupplies[i - 1]);
          int ip = abs(resupplies[i]);
          for (int j = im + 1; j <= ip; j++)
            speeds[j] = resupSpeeds[i];
        }
      }
    }

    score = 0.0;
    fuelLeft = MaxFuel, oxygenLeft = MaxOxygen, foodLeft = MaxFood;
    for (int j = 1; j < resupplyCount; j++)
    {
      int i = abs(resupplies[j]);
      int ip = abs(resupplies[j - 1]);
      int node = path[i];
      double distance = resupDists[j];
      if (j < resupplyCount - 1) //Iterate through each refill node in path and optimize speed for segment
      {
        if (optimizeSpeed(ip + 1, i, pathLength - 1, path, mb, speeds, fuel, oxygen, food) == -1)
        {
          error = -1;
          score = INF;
          break;
        }
      }
      for (int m = j; m < resupplyCount; m++)
      {
        resupSpeeds[m] = speeds[abs(resupplies[m])];
      }
      double speed = speeds[i];
      double t = distance / speed;
      double consumed = Kl * t;
      fuelLeft -= Kf * distance * speed * speed, oxygenLeft -= consumed, foodLeft -= consumed;
      rawOxygen[i] = oxygenLeft, rawFood[i] = foodLeft;
      if (fuelLeft <= 0 || oxygenLeft <= 0 || foodLeft <= 0)
      {
        error = -1;
        score = INF;
        break;
      }
      switch (planets[node].resupply)
      {
      case OXYGEN:
        oxygenLeft = MaxOxygen;
        break;
      case FOOD:
        foodLeft = MaxFood;
        break;
      default:
        break;
      }
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      score += t;
    }
    score -= fuel.back() + oxygen.back() + food.back();
    if (score > prevScore)
    {
      dsc /= dsr;
      speeds = speeds0;
      fuel = fuel0;
      oxygen = oxygen0;
      food = food0;
      fuelLeft = fuelLeft0;
      oxygenLeft = oxygenLeft0;
      foodLeft = foodLeft0;
      score = prevScore;
      if (dsc < 0.1)
        break;
    }
    else if (iter > 0)
    {
      dsc *= dsr;
      if (dsc > 10.0)
        break;
    }
    prevScore = score;
  }

  error = calculateResources(path, speeds, fuel, oxygen, food, rawOxygen, rawFood, score);
  return error;
}

void printData(const vector<int> &path, const vector<double> &speeds, const vector<double> &fuel, const vector<double> &oxygen,
               const vector<double> &food, const vector<double> &rawOxygen, const vector<double> &rawFood)
{
  //Determine formatting width for each stat
  int pathLength = path.size();
  if (pathLength < 2)
  {
    cout << "Empty Path in Data" << endl;
    return;
  }

  int indexWidth = (int)log10(pathLength - 1) + 1;
  if (longestNameLength < 6)
    longestNameLength = 6;
  vector<string> speedStrings(pathLength);
  int longestSpeedLength = 0;
  for (int i = 0; i < pathLength; i++)
  {
    stringstream ss;
    ss << fixed << setprecision(14) << speeds[i];
    string speedString = ss.str();
    speedStrings[i] = (speedString);
    if (speedString.size() > longestSpeedLength)
      longestSpeedLength = speedString.size();
  }

  cout << endl << endl << "PROGRAM OUTPUT" << endl;
  cout << "Map name: " << mapName << endl;
  cout << "Planet travel order from " << planets[0].name << " (excluded) to " << planets[lastNodeIndex].name << " (included)" << endl << endl;
  cout << right << setw(longestNameLength + indexWidth + 5) << "Planet |" << setw(longestSpeedLength + 3) << "Speed To Planet |" << setw(12) << "Distance |" << setw(12) << "Time |" << setw(12) << "Fuel |" << setw(10) << "Oxygen" << setw(8) << "Res. |" << setw(10) << "Food" << setw(8) << "Res. |" << setw(13) << "Score |" << endl;
  int width = longestNameLength + indexWidth + longestSpeedLength + 93;
  cout << scientific << setprecision(3);
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
      o = "->" + to_string((int)MaxOxygen);
      break;
    case FOOD:
      fo = "->" + to_string((int)MaxFood);
      break;
    default:
      break;
    }
    if (edgeInHostileRadius(path[i - 1], node))
      cout << "* Edge crosses hostile radius" << endl;
    cout << setw(indexWidth) << i << ")  "
         << setw(longestNameLength) << planet.name << " | "
         << setw(longestSpeedLength) << speedStrings[i] << " | "
         << setw(9) << d << " | " //Scientific notation for consistent formatting
         << setw(9) << travelTime << " | "
         << setw(9) << fu << " | "
         << setw(9) << rawOxygen[i]
         << setw(6) << o << " | "
         << setw(9) << rawFood[i]
         << setw(6) << fo << " | "
         << setw(10) << s << " |"
         << endl;
  }
  cout << "* Note: Some values are in scientific notation for consistent formatting (e.g. 6.67e-02 = 0.0667)" << endl;
  cout << endl << "Final stats" << endl;
  double fuelLeft = fuel.back(), oxygenLeft = oxygen.back(), foodLeft = food.back();
  cout << "Total time: " << defaultfloat << setprecision(8) << travelTime << endl;
  cout << "Total Distance: " << travelDistance << endl;
  cout << "Remaining fuel: " << fuelLeft << endl;
  cout << "Remaining oxygen: " << oxygenLeft << endl;
  cout << "Remaining food: " << foodLeft << endl;
  cout << "Score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

#pragma endregion

#pragma region TESTING/DEV FEATURES

//Writes random map to RandomMap.csv with specified parameters (testing pathfinding/resources)
int generateRandomMap(bool useLast = false)
{
  cout << endl;
  int planetNumber;
  double maxPos, maxRad, hosProb, oxygenProb, foodProb;
  if (useLast)
    planetNumber = lastPN, maxPos = lastMP, maxRad = lastHR, hosProb = lastHP, oxygenProb = lastOP, foodProb = lastFP;
  else
  {
    cout << "Input parameters for random map generation" << endl;
    string input;
    cout << "Planet number (return for default 50): ";
    getline(cin, input);
    if (input == "")
      input = "50";
    planetNumber = stoi(input);
    cout << "Max pos (return for default 100): ";
    getline(cin, input);
    if (input == "")
      input = "100";
    maxPos = stod(input);
    cout << "Max hostile radius (return for default 10): ";
    getline(cin, input);
    if (input == "")
      input = "10";
    maxRad = stod(input);
    cout << "Hostile probability (return for default 0.1): ";
    getline(cin, input);
    if (input == "")
      input = "0.1";
    hosProb = stod(input);
    cout << "Oxygen resupply probability (return for default 0.1): ";
    getline(cin, input);
    if (input == "")
      input = "0.1";
    oxygenProb = stod(input);
    cout << "Food resupply probability (return for default 0.1): ";
    getline(cin, input);
    if (input == "")
      input = "0.1";
    foodProb = stod(input);
    lastPN = planetNumber, lastMP = maxPos, lastHR = maxRad, lastHP = hosProb, lastOP = oxygenProb, lastFP = foodProb;
  }

  random_device rd;
  mt19937 engine(rd());
  uniform_real_distribution<double> xPos(-maxPos, maxPos);
  uniform_real_distribution<double> yPos(-maxPos, maxPos);
  uniform_real_distribution<double> res(0, 1);
  uniform_real_distribution<double> hos(0, 1);
  uniform_real_distribution<double> rad(0.1, maxRad);

  ofstream outFile("RandomMap.csv");
  if (!outFile.is_open())
  {
    cout << "Error outputting map to file RandomMap.csv" << endl;
    return -1;
  }
  outFile << "Planet Name,x,y,type,resupply,hostile_radius" << endl;
  for (int i = 0; i < planetNumber; i++)
  {
    double x = xPos(engine), y = yPos(engine);
    if (i == 0)
      x = y = -maxPos/1.5;
    if (i == planetNumber - 1)
      x = y = maxPos/1.5;

    double rand = res(engine);
    string resupply = "";
    if (i != 0 && i != planetNumber - 1) //Ensure start/target are not resupplies
    {
      if (rand < oxygenProb)
        resupply = "Oxygen";
      else if (rand < oxygenProb + foodProb)
        resupply = "Food";
    }

    string hosType;
    double hosRad;
    //Ensure start/target are neutral and outside hostile radius
    if (hos(engine) <= hosProb && i != 0 &&
        i != planetNumber - 1 &&
        calculateDistance(Position(x, y), Position(-maxPos/1.5, -maxPos/1.5)) > maxRad &&
        calculateDistance(Position(x, y), Position(maxPos/1.5, maxPos/1.5)) > maxRad)
    {
      hosType = "Hostile";
      hosRad = rad(engine);
    }
    else
    {
      hosType = "Neutral";
      hosRad = 0;
    }
    outFile << i << "," << x << "," << y << "," << hosType << "," << resupply << "," << hosRad << endl;
  }
  outFile.close();
  cout << "Map data successfully outputted to RandomMap.csv" << endl;
  return 0;
}

//Takes path input, optimizes speed and outputs score data (testing path optimality)
int testPath(string &xPath, string &yPath)
{
  string input;
  cout << endl
       << "Path (excluding " << planets[0].name << ", return to exit): ";
  getline(cin, input);
  if (input.empty())
    return -1;
  cout << endl;
  input = planets[0].name + "," + input;
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
    printData(path, speeds, fuel, oxygen, food, rawOxygen, rawFood);
  else
    cout << "Invalid path (resources run out)" << endl;
  for (int i = 0; i < pathLength; i++)
  {
    xPath += "," + to_string(planets[path[i]].position.x);
    yPath += "," + to_string(planets[path[i]].position.y);
  }
  return 0;
}

//Takes path and speed input, outputs raw score data (testing data calcs)
int rawTestPath(string &xPath, string &yPath)
{
  string input, input2;
  cout << endl
       << "Path (excluding start, return to exit): ";
  getline(cin, input);
  if (input.empty())
    return -1;
  cout << "Speeds (return to exit): ";
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
      previousNode[0] = MaxFuel;
      previousNode[1] = MaxOxygen;
      previousNode[2] = MaxFood;
    }
    //Use raw calculated distance (not cached) for testing
    double v = speeds[i], t, o, fo, d = calculateDistance(planets[node0].position, planet.position);
    switch (planet.resupply)
    {
    case OXYGEN:
      t = d / v;
      o = MaxOxygen;
      fo = previousNode[2] - Kl * t;
      break;
    case FOOD:
      t = d / v;
      fo = MaxFood;
      o = previousNode[1] - Kl * t;
      break;
    default:
      t = d / v;
      o = previousNode[1] - Kl * t, fo = previousNode[2] - Kl * t;
      break;
    }
    double fu = previousNode[0] - Kf * d * v * v;
    travelTime += t;
    double s = travelTime - (fu + o + fo);
    array<double, 3> tnode = array<double, 3>();
    tnode[0] = fu;
    tnode[1] = o;
    tnode[2] = fo;
    pathData.push_back(tnode);
    if (edgeInHostileRadius(node0, node))
      cout << "* Edge crosses hostile radius" << endl;
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

//Prints a path (used in debugging)
void printPath(const string id, const vector<int> &path, const double score)
{
  if (id.length() > 0)
    cout << id << ": ";
  for (int i : path)
    cout << i << ", ";
  cout << score << endl;
}

#pragma endregion

int main(int argc, char *argv[])
{
  //Command line options for terminal input
  //e.g. ./ExplorationAlgorithm --map=ExampleMap.csv
  int opt, index;
  static struct option long_options[] =
  {
    {"map", required_argument, 0, 'm'},       //Input map file name (with extension)
    {"batchSize", required_argument, 0, 'b'}, //Paths to generate each batch, defaults to 5 * nodeCount (larger batch improves accuracy but costs performance)
    {"testPath", required_argument, 0, 't'},  //Tests inputted path and speeds (1) or tests and optimizes speeds on inputted path (2) or both (3)
    {"plotMap", required_argument, 0, 'p'},   //If 1, outputs map and path data to PlotterData.csv for testing (ensure file GraphPlotter.py is in directory and python3 is installed on machine)
    {"randomMap", required_argument, 0, 'r'}, //If 1, generates random map in RandomMap.csv with specified paremeters
    {"distScale", required_argument, 0, 'd'}, //Scales all planet distances by constant factor distScale (used to simulate extreme conditions)
    {"quickTest", required_argument, 0, 'q'}, //Loops random generation and algorithm for rapid testing (1 for auto-input, 2 for manual)
    {0, 0, 0, 0}
  };
  while ((opt = getopt_long(argc, argv, "m:b:t:p:r:d:q:", long_options, &index)) != -1)
  {
    switch (opt)
    {
    case 'm':
      fileName = optarg;
      break;
    case 'b':
      batchSize = atoi(optarg);
      break;
    case 't':
      inputPath = atoi(optarg);
      break;
    case 'p':
      plotMap = atoi(optarg);
      break;
    case 'r':
      randomMap = atoi(optarg);
      break;
    case 'd':
      distScale = atof(optarg);
      break;
    case 'q':
      quickTest = atoi(optarg);
      break;
    default:
      break;
    }
  }

  cout << endl << "Welcome Mission Control!" << endl;
  int counter = 0;
  while (quickTest || counter == 0)
  {
    if (randomMap > 0)
    {
      if (counter == 0 || quickTest == 2)
        generateRandomMap();
      else
        generateRandomMap(true);
    }
    counter ++;
    if (fileName.empty())
    {
      cout << "Use these instructions to begin stress-testing:" << endl;
      cout << endl;
      cout << "This program expects a .csv map file formatted as per the challenge guide (Planet Name,x,y,type,resupply,hostile_radius)." << endl;
      cout << "It will output the best route from the first planet in the file (right after the header line) to the planet on the last line of the file." << endl;
      cout << endl;
      cout << "Put the CSV file in the same directory/folder as this program and then simply input the file name below (e.g. ExampleMap.csv)." << endl;
      cout << "If the map cannot be placed in this folder, the full file path can be provided:" << endl;
      cout << "e.g. C:\\Users\\Your Username\\Folder Name\\ExampleMap.csv on Windows or /Users/Your Username/Folder Name/ExampleMap.csv on Mac." << endl;
    }

    //Input and setup
    int error;
    do
    {
      planets.clear();
      hostilePlanets.clear();
      error = parseCSV();
      if (error != 0) fileName = "";
    } while (error != 0);

    //Pathfinding and speed/resource optimization
    initializeLists();
    if (batchSize <= 0)
    {
      batchSize = (int) 3 * nodeCount;
      if (batchSize > 500)
        batchSize = 500;
    }
    vector<vector<int>> bestPaths;
    vector<int> bestPath, fbPath; //Fallback path if no valid best path is found
    vector<double> bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood; //Raw = pre-resupply
    //int bestIndex;
    bool validPath = true;
    //Finds batch of size pathCount best paths, optimizes speed for each path in bestPaths and selects the best out of them for output
    //If no valid path is found, generates another batch of next pathCount best paths until valid path is found or 100000 paths searched
    int maxIterations = 100000 / batchSize; //Cap prevents infinite loop, should time out ~2min
    for (int i = 0; i < maxIterations; i++)
    {
      double bestScore = INF;
      if (yens(bestPaths, batchSize) == -1)
      {
        validPath = false;
        if (fbPath.empty())
        {
          cout << endl << "A valid path could not be found. Please try a different map." << endl;
          break;
        }
        calculateResources(fbPath, bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood, bestScore);
        printData(fbPath, bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood);
        cout << endl << "INVALID PATH: Please try a different map." << endl;
        break;
      }
      else
      {
        double score;
        int startBatch = batchSize * i, endBatch = min(startBatch + batchSize, (int) bestPaths.size());
        for (int j = startBatch; j < endBatch; j++) //Only optimize out of paths in current batch for performance
        {
          vector<int> &path = bestPaths[j];
          int pathLength = path.size();
          if (pathLength < 2)
            continue;
          vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength), rawOxygen(pathLength), rawFood(pathLength);
          fuel[0] = MaxFuel, oxygen[0] = MaxOxygen, food[0] = MaxFood;
          score = INF;
          error = optimizePath(path, score, speeds, fuel, oxygen, food, rawOxygen, rawFood);
          if (error == 0 && score < bestScore)
            bestScore = score, bestPath = path, bestSpeeds = speeds, bestFuel = fuel, bestOxygen = oxygen, bestFood = food, bestRawOxygen = rawOxygen, bestRawFood = rawFood;//, bestIndex = j;
        }
        if (bestScore < INF)
        {
          printData(bestPath, bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood);
          //cout << "Paths searched: " << bestIndex+1 << endl;
          break;
        }
        else if (i == 0)
          fbPath = bestPaths.back();
        else if (i == maxIterations - 1)
        {
          validPath = false;
          if (fbPath.empty())
          {
            cout << endl << "A valid path could not be found. Please try a different map." << endl;
            break;
          }
          calculateResources(fbPath, bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood, bestScore);
          printData(fbPath, bestSpeeds, bestFuel, bestOxygen, bestFood, bestRawOxygen, bestRawFood);
          cout << endl << "INVALID PATH: Please try a different map." << endl;
        }
      }
    }
    long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
    double milliseconds = calcTime / 1000.0;
    cout << endl
        << "Calculation time: " << fixed << setprecision(6) << milliseconds / 1000 << " seconds (" << setprecision(3) << milliseconds << " milliseconds)" << endl;

    //Output map and path data to PlotterData.csv for testing
    if (plotMap > 0)
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
      ofstream outFile("PlotterData.csv");
      if (outFile.is_open())
      {
        outFile << xEndpt.substr(1) << endl;
        outFile << yEndpt.substr(1) << endl;
        if (!xNone.empty())
          outFile << xNone.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!yNone.empty())
          outFile << yNone.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!xOxygen.empty())
          outFile << xOxygen.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!yOxygen.empty())
          outFile << yOxygen.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!xFood.empty())
          outFile << xFood.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!yFood.empty())
          outFile << yFood.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!xHostile.empty())
          outFile << xHostile.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!yHostile.empty())
          outFile << yHostile.substr(1) << endl;
        else
          outFile << "," << endl;
        if (!rHostile.empty())
          outFile << rHostile.substr(1) << endl;
        else
          outFile << "," << endl;
        if (validPath)
        {
          outFile << xPath.substr(1) << endl;
          outFile << yPath.substr(1) << endl;
        }
        else
        {
          outFile << "," << endl;
          outFile << "," << endl;
        }
        //Tests inputted path and speeds (inputPath=1,3) or tests and optimizes speeds on inputted path (inputPath=2,3)
        //Adds tested paths to plot data as well
        while (inputPath == 1 || inputPath == 3)
        {
          xPath = to_string(planets[0].position.x), yPath = to_string(planets[0].position.y);
          if (rawTestPath(xPath, yPath) == -1)
            break;
          outFile << xPath << endl;
          outFile << yPath << endl;
        }
        while (inputPath == 2 || inputPath == 3)
        {
          xPath = "", yPath = "";
          if (testPath(xPath, yPath) == -1)
            break;
          outFile << xPath.substr(1) << endl;
          outFile << yPath.substr(1) << endl;
        }
        outFile.close();
      }
      else
      {
        cout << "Error outputting map data to file PlotterData.csv" << endl;
        cout << endl;
      }

      cout << endl;
      if (system("python3 ./GraphPlotter.py") == 0)
        cout << "Successfully ran GraphPlotter.py" << endl;
      else
        cout << "Error running GraphPlotter.py." << endl;
    }
    else
    {
      //Tests inputted path and speeds (inputPath=1,3) or tests and optimizes speeds on inputted path (inputPath=2,3)
      while (inputPath == 1 || inputPath == 3)
      {
        string xPath = "", yPath = "";
        if (rawTestPath(xPath, yPath) == -1)
          break;
      }
      while (inputPath == 2 || inputPath == 3)
      {
        string xPath = "", yPath = "";
        if (testPath(xPath, yPath) == -1)
          break;
      }
    }

    if (quickTest == 1)
    {
      cout << endl << "Return to continue: ";
      string str;
      getline(cin, str);
      if (str.empty())
        continue;
      else
        break;
    }
    else
      break;
  }
}
