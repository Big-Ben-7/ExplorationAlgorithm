#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#define Kf 0.15
#define Kl 0.3
#define MAX_FUEL 3000
#define MAX_OXYGEN 2000
#define MAX_FOOD 2000
#define infinity (numeric_limits<double>::infinity())

using namespace std;

// Use enum instead of string for resupply to improve performance
enum class Resupply { NONE, OXYGEN, FOOD };
struct Point
{
   double x, y;
};
struct PlanetData
{
   string name;
   Point position;
   Resupply resupply;
   double hostileRadius;
};
// Edges defined by starting node/vertex (planet), ending node, and distance
struct Edge
{
   int node1, node2;
   double distance;
};
// Stores cost, resource consumption, and order of travel and speed for best path to node
struct NodeData
{
   double minCost, fuel, food, oxygen, time;
   vector<int> bestPath;
   vector<double> speeds;
   NodeData(int initialCost)
   {
      minCost = initialCost;
      fuel = MAX_FUEL;
      oxygen = MAX_OXYGEN;
      food = MAX_FOOD;
      time = 0;
      bestPath = vector<int>();
      speeds = vector<double>();
   }
};
vector<vector<double>> nodeDistances;
vector<PlanetData> planets;
int nodeCount, edgeCount;
chrono::steady_clock::time_point startTime;
// Edge list used to efficiently represent graph
vector<Edge> edgeList;
vector<NodeData> nodes;

// Speed optimization by first derivative test
// Cost = c(x) = time + fuel + food + oxygen = d/v + Kf*d*v^2 + Kl*d/v + Kl*d/v
// c'(x) = -d/v^2 + 2*Kf*d*v - Kl*d/v^2 - Kl*d/v^2 = (-d + 2*Kf*d*v^3 - Kl*d - Kl*d)/v^2
// Cost minimized at c'(x) = 0 => -d + 2*Kf*d*v^3 - Kl*d - Kl*d = 0 => v = cbrt((1 + Kl+Kl) / (2*Kf))
double idealSpeed = cbrt((1 + 2 * Kl) / (2 * Kf));
// Ideal speed disregarding consumption of one Kl term (one resupply remaining in path)
double singleResupplySpeed = cbrt((1 + Kl) / (2 * Kf));
// Ideal speed disregarding consumption of both Kl terms (both resupply types remaining)
double doubleResupplySpeed = 1 / cbrt(2 * Kf);
string mapName;


string toLower(string str)
{
   transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {return tolower(c);});
   return str;
}
string trim(string str)
{
   str.erase(str.begin(), find_if(str.begin(), str.end(), [](unsigned char c) {return !isspace(c);}));
   str.erase(find_if(str.rbegin(), str.rend(), [](unsigned char c) {return !isspace(c);}).base(), str.end());
   return str;
}
bool isValidPlanet(string planetName)
{
   bool alphanumeric;
   for (char c : planetName)
   {
      if (isalnum(c))
      {
         if (!alphanumeric) alphanumeric = true;
         continue;
      }
      switch (c)
      {
         // Spaces, dashes, and parenthesis allowed
         case ' ': case '-': case '(': case ')': continue;
         default: return false;
      }
   }
   // Planet names can't be all special characters!
   return alphanumeric;
}
double calculateDistance(Point pos1, Point pos2)
{
   double dx = pos2.x - pos1.x;
   double dy = pos2.y - pos1.y;
   return sqrt(dx * dx + dy * dy);
}

int parseCSV()
{
   // File input
   cout << endl;
   ifstream iFile;
   string fileName, line = "";
   cout << "Map file name: ";
   getline(cin, fileName);
   mapName = fileName.substr(0, fileName.rfind('.'));
   // Start computation time immediately after user enters file name.
   startTime = chrono::steady_clock::now();
   iFile.open(fileName);
   if (!iFile.is_open())
   {
      cout << "Error: file not found. Please make sure that the entered name is correct and that the CSV file is in the same folder as this project." << endl;
      return -1;
   }

   // Header line error handling
   do
   {
      getline(iFile, line);
   } while (trim(line) == ""); // Skip empty lines
   if (count(line.begin(), line.end(), ',') != 5)
   {
      cout << "Error: Line 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
      return -1;
   }
   int commaIndex;
   string headerSections[] = {"planet name", "x", "y", "type", "resupply", "hostile_radius"};
   for (int i = 0; i < 5; i++)
   {
      commaIndex = line.find(',');
      if(trim(toLower(line.substr(0, commaIndex))) != headerSections[i])
      {
         cout << "Error: Line 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
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
      if (trim(line) == "") continue; // Skip empty lines
      lineNumber ++;
      int extraCommas = count(line.begin(), line.end(), ',') - 5;
      if (extraCommas > 0)
      {
         if (extraCommas == 1) cout << "Error: Line " << lineNumber << " has an extra comma." << endl;
         else cout << "Error: Line " << lineNumber << " has " << extraCommas << " extra commas." << endl;
         return -1;
      }
      else if (extraCommas < 0)
      {
         extraCommas *= -1;
         if (extraCommas == 1) cout << "Error: Line " << lineNumber << " needs an extra comma." << endl;
         else cout << "Error: Line " << lineNumber << " needs " << extraCommas << " extra commas." << endl;
         return -1;
      }
      
      // Planet name
      commaIndex = line.find(',');
      planetName = trim(line.substr(0, commaIndex));
      if (planetName == "" || !isValidPlanet(planetName))
      {
         cout << "Error: \"" << planetName << "\" (line " << lineNumber << ") is an invalid planet name. Planet names must be alphanumeric (spaces, dashes, and parenthesis accepted)." << endl;
         return -1;
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
         cout << "Error: The x-coordinate \"" << substring << "\" of planet \"" << planetName << "\" (line " << lineNumber << ") must be numerical." << endl;
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
         cout << "Error: The y-coordinate \"" << substring << "\" of planet \"" << planetName << "\" (line " << lineNumber << ") must be numerical." << endl;
         return -1;
      }
      line = line.substr(commaIndex + 1);
      
      // Type
      commaIndex = line.find(',');
      type = trim(toLower(line.substr(0, commaIndex)));
      if (type != "neutral" && type != "hostile")
      {
         cout << "Error: The type \"" << type << "\" of planet \"" << planetName << "\" (line " << lineNumber << ") must be \"neutral\" or \"hostile\"." << endl;
         return -1;
      }
      line = line.substr(commaIndex + 1);
      
      // Resupply
      commaIndex = line.find(',');
      resupply = trim(toLower(line.substr(0, commaIndex)));
      Resupply resupplyEnum;
      if (resupply == "oxygen") resupplyEnum = Resupply::OXYGEN;
      else if (resupply == "food") resupplyEnum = Resupply::FOOD;
      else if (resupply == "" || resupply == "none") resupplyEnum = Resupply::NONE;
      else
      {
         cout << "Error: The resupply \"" << resupply << "\" of planet \"" << planetName << "\" (line " << lineNumber << ") must be \"\" (none), \"food\", or \"oxygen\"." << endl;
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
         cout << "Error: The hostile radius \"" << line << "\" of planet \"" << planetName << "\" (line " << lineNumber << ") must be numerical." << endl;
         return -1;
      }
      if (type == "neutral" && hostileRadius != 0)
      {
         cout << "Error: The hostile radius \"" << hostileRadius << "\" of neutral planet \"" << planetName << "\" (line " << lineNumber << ") must be 0." << endl;
         return -1;
      }
      else if (type == "hostile" && hostileRadius <= 0)
      {
         cout << "Error: The hostile radius \"" << hostileRadius << "\" of hostile planet \"" << planetName << "\" (line " << lineNumber << ") must be positive." << endl;
         return -1;
      }
      planets.push_back({planetName, {x, y}, resupplyEnum, hostileRadius});
   }
   nodeCount = lineNumber - 1;
   iFile.close();
   return 0;
}

bool edgeInHostileRadius(int startNode, int endNode)
{
   // Calculate intersection of line through (x1,y1) and (y1,y2) with circle at (x3,y3) and radius r
   Point& start = planets[startNode].position, end = planets[endNode].position;
   // Line eq. (parametric): x = x1 + cos(theta)*d, y = y1 + sin(theta)*d where d=distance along line and theta = atan(slope)
   double x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y, rise = y2-y1, run = x2-x1;
   double theta = atan2(rise, run), cosTheta = cos(theta), sinTheta = sin(theta); // Precompute trig for efficiency
   double l = run*run + rise*rise; // Length of line
   for (int i = 0; i < nodeCount; i++)
   {
      if (planets[i].hostileRadius == 0) continue;
      Point& center = planets[i].position;
      // Circle eq.: (x-x3)^2 + (y-y3)^2 = r^2
      double x3 = center.x, y3 = center.y, r = planets[i].hostileRadius, dx = x1-x3, dy = y1-y3;
      // Plugging line eqs. into circle eq. and applying trig identities yields quadratic a*d^2 + b*d + c = 0
      double b = 2*dx*cosTheta + 2*dy*sinTheta, c = dx*dx + dy*dy - r*r; // a=1
      double discriminant = b*b - 4*c;
      // If discriminant < 0, no solution and edge does not cross hostile circle
      if (discriminant < 0) continue;
      double d1 = (-b + sqrt(discriminant)) / 2, d2 = (-b - sqrt(discriminant)) / 2; // Roots from quadratic formula
      // If both values for d are lower or higher than edge length, hostile circle is still not crossed
      if ((d1 < 0 && d2 < 0) || (d1*d1 > l && d2*d2 > l)) continue;
      return true;
   }
   return false;
}

void initializeLists()
{
   // Set distance to initial node to 0
   nodes.emplace_back(0);
   nodeDistances = vector<vector<double>>(nodeCount, vector<double>(nodeCount));
   for (int i = 0; i < nodeCount; i++)
   {
      if (i != 0)
      {
         // Set pre-relaxed distance to all other nodes to infinity
         nodes.emplace_back(infinity);
      }
      if (planets[i].hostileRadius != 0) continue;
      // Fill edge list with edges for each pair of connected nodes
      for (int j = i + 1; j < nodeCount; j++)
      {
         // Each planet connected to all other non-hostile planets
         if (edgeInHostileRadius(i, j)) continue;
         double distance = calculateDistance(planets[i].position, planets[j].position);
         edgeList.push_back({i, j, distance});
         edgeList.push_back({j, i, distance});
         nodeDistances[i][j] = nodeDistances[j][i] = distance;
      }
   }
   edgeCount = edgeList.size();
}

// Stage 1: Implement modified Bellman-Ford algorithm to find optimal path
void calculatePath()
{
   bool unrelaxed = true;
   // Terminates once one iteration has passed with no relaxations
   while (unrelaxed)
   {
      unrelaxed = false;
      for (int i = 0; i < edgeCount; i++)
      {
         Edge& edge = edgeList[i];
         int node1 = edge.node1, node2 = edge.node2;
         NodeData& node1Data = nodes[node1];
         NodeData& node2Data = nodes[node2];
         double d = edge.distance, speed, t;
         double currentFuel = node1Data.fuel, currentOxygen = node1Data.oxygen, currentFood = node1Data.food;
         
         // Dynamically assign weights (travel cost) based on time, resource consumption, and resupplies
         // Resupplies can cause negative weights (negative cycles vanish over iterations, so no infinite loops)
         double oxygenConsumed, foodConsumed, oxygenLeft, foodLeft;
         switch (planets[node2].resupply)
         {
            case Resupply::OXYGEN:
               speed = singleResupplySpeed, t = d/speed;
               oxygenConsumed = currentOxygen - MAX_OXYGEN;
               oxygenLeft = MAX_OXYGEN;
               foodConsumed = Kl*t;
               foodLeft = currentFood - foodConsumed;
               break;
            case Resupply::FOOD:
               speed = singleResupplySpeed, t = d/speed;
               foodConsumed = currentFood - MAX_FOOD;
               foodLeft = MAX_FOOD;
               oxygenConsumed = Kl*t;
               oxygenLeft = currentOxygen - oxygenConsumed;
               break;
            case Resupply::NONE:
               speed = idealSpeed, t = d/speed;
               oxygenConsumed = foodConsumed = Kl*t;
               oxygenLeft = currentOxygen - oxygenConsumed;
               foodLeft = currentFood - foodConsumed;
               break;
         }
         double fuelConsumed = Kf*d*speed*speed, fuelLeft = currentFuel - fuelConsumed;
         double newCost = node1Data.minCost + t + fuelConsumed + oxygenConsumed + foodConsumed;

         if (newCost < node2Data.minCost)
         {
            // Relax edge
            node2Data.minCost = newCost;
            node2Data.fuel = fuelLeft;
            node2Data.food = foodLeft;
            node2Data.oxygen = oxygenLeft;
            node2Data.time = node1Data.time + t;
            node2Data.speeds = node1Data.speeds;
            node2Data.speeds.push_back(speed);
            node2Data.bestPath = node1Data.bestPath;
            node2Data.bestPath.push_back(node2);
            if (!unrelaxed) unrelaxed = true;
         }
      }
   }
}

// Stage 2: Optimize resource usage given the known path
int optimizePath(vector<int>& path, vector<double>& speeds, vector<double>& fuel, vector<double>& oxygen, vector<double>& food)
{
   int pathLength = path.size();
   int lastFoodResupply = -1, lastOxygenResupply = -1;
   for (int i = pathLength - 1; i >= 0; i--)
   {
      switch (planets[path[i]].resupply)
      {
         case Resupply::OXYGEN:
            if (lastOxygenResupply == -1) lastOxygenResupply = i;
            break;
         case Resupply::FOOD:
            if (lastFoodResupply == -1) lastFoodResupply = i;
            break;
         case Resupply::NONE:
            break;
      }
      if (lastOxygenResupply != -1 && lastFoodResupply != -1) break;
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
      totalFuelConsumed += Kf * nodeDistances[node0][node] * doubleResupplySpeed * doubleResupplySpeed;
      node0 = node;
   }
   for (int i = resupply1 + 1; i <= resupply2; i++)
   {
      // Before last oxygen/food refill, optimize speed based on fuel and food/oxygen
      speeds[i] = singleResupplySpeed;
      int node = path[i];
      totalFuelConsumed += Kf * nodeDistances[node0][node] * singleResupplySpeed * singleResupplySpeed;
      node0 = node;
   }
   for (int i = resupply2 + 1; i < pathLength; i++)
   {
      // After refills, optimize speed based on fuel, food, and oxygen
      speeds[i] = idealSpeed;
      int node = path[i];
      totalFuelConsumed += Kf * nodeDistances[node0][node] * idealSpeed * idealSpeed;
      node0 = node;
   }
   // If fuel drops to 0, reduce speed by constant factor
   double speedFactor = sqrt((MAX_FUEL - 0.001) / totalFuelConsumed);
   double fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
   node0 = 0;
   if (totalFuelConsumed >= MAX_FUEL)
   {
      for (int i = 0; i < pathLength; i++)
      {
         int node = path[i];
         speeds[i] *= speedFactor;
         double speed = speeds[i], distance = nodeDistances[node0][node], t = distance / speed;
         double consumed = Kl * t, fuelConsumed = Kf * distance * speed * speed;
         fuelLeft -= fuelConsumed;
         // If oxygen or food then drop to 0, path is invalid because food/oxygen and fuel have opposite dependencies on speed
         if (consumed > foodLeft - 0.001 || consumed > oxygenLeft - 0.001) return -1;
         switch (planets[node].resupply)
         {
            case Resupply::OXYGEN:
               oxygenLeft = MAX_OXYGEN, foodLeft -= consumed;
               break;
            case Resupply::FOOD:
               foodLeft = MAX_FOOD, oxygenLeft -= consumed;
               break;
            case Resupply::NONE:
               oxygenLeft -= consumed, foodLeft -= consumed;
               break;
         }
         fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
         node0 = node;
      }
   } 
   else 
   {
      for (int i = 0; i < pathLength; i++)
      {
         int node = path[i];
         double distance = nodeDistances[node0][node], speed = speeds[i];
         double minOxygenSpeed = distance / ((oxygenLeft - 0.01) / Kl);
         double minFoodSpeed = distance / ((foodLeft - 0.01) / Kl);
         double minSpeed = min(minOxygenSpeed, minFoodSpeed), maxSpeed = speedFactor * speed;
         if (minSpeed > maxSpeed) return -1;
         if (speed < minSpeed) speeds[i] = speed = minSpeed;
         double consumed = Kl * distance / speed, fuelConsumed = Kf * distance * speed * speed;
         fuelLeft -= fuelConsumed;
         switch (planets[node].resupply)
         {
            case Resupply::OXYGEN:
               oxygenLeft = MAX_OXYGEN, foodLeft -= consumed;
               break;
            case Resupply::FOOD:
               foodLeft = MAX_FOOD, oxygenLeft -= consumed;
               break;
            case Resupply::NONE:
               oxygenLeft -= consumed, foodLeft -= consumed;
               break;
         }
         fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
         node0 = node;
      }
   }
   return 0;
}

void printData(vector<int>& path, vector<double>& speeds, vector<double>& fuel, vector<double>& oxygen, vector<double>& food)
{
   cout << endl;
   cout << "Map used: " << mapName << endl;
   cout << "Planet travel order (stats upon arrival, all values in AU): " << endl;
   cout << endl;
   cout << " " << right << setw(12) << "Planet: " << setw(12) << "Speed: " << setw(12) << "Dist: " << setw(12) << "Time: " << setw(12) << "Fuel: " << setw(12) << "Oxyg: " << setw(12) << "Food: " << setw(12) << "Score: " << endl;
   cout << fixed << setprecision(4);
   int pathLength = path.size();
   int node0 = 0;
   double travelTime = 0;
   for (int i = 0; i < pathLength; i++)
   {
      int node = path[i];
      double d = nodeDistances[node0][node], v = speeds[i], t = d/v;
      double fu = fuel[i], o = oxygen[i], fo = food[i], s = t - (fu + o + fo);
      if (edgeInHostileRadius(node0, node)) cout << "DEBUG: INVALID PATH - edge crosses hostile radius" << endl;
      cout << i + 2 << ") " 
           << right
           << setw(9) << planets[node].name << "   " 
           << setw(9) << v << "   " 
           << setw(9) << d << "   " 
           << setw(9) << t << "   " 
           << setw(9) << fu << "   " 
           << setw(9) << o << "   " 
           << setw(9) << fo << "   " 
           << setw(9) << s
           << endl;
      travelTime += t;
      node0 = node;
   }
   cout << endl;
   cout << "Final stats: " << endl;
   double fuelLeft = fuel.back(), oxygenLeft = oxygen.back(), foodLeft = food.back();
   cout << "Travel time: " << travelTime << endl;
   cout << "Remaining fuel: " << fuelLeft << endl;
   cout << "Remaining oxygen: " << oxygenLeft << endl;
   cout << "Remaining food: " << foodLeft << endl;
   cout << "Score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

void testPath()
{
   cout << endl;
   string input;
   cout << "Path: ";
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
         if (planets[i].name != planet) continue;
         path.push_back(i);
         break;
      }
   }
   string planet = trim(input);
   for (int i = 0; i < planets.size(); i++)
   {
      if (planets[i].name != planet) continue;
      path.push_back(i);
      break;
   }
   int pathLength = commaCount + 1;
   vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength);
   if (optimizePath(path, speeds, fuel, oxygen, food) == 0) printData(path, speeds, fuel, oxygen, food);
}

int main()
{
   cout << endl;
   cout << "Welcome Mission Control!" << endl;
   cout << endl;
   cout << "Use these instructions to begin stress-testing:" << endl;;
   cout << "Put the map CSV file in the same directory/folder as this program and then simply input the file name below (e.g., ExampleMap.csv)." << endl;
   cout << "If the map cannot be placed in the same folder, the full file path can be provided:" << endl;
   cout << "e.g., C:\\Users\\Your Username\\Folder Name\\ExampleMap.csv on Windows or /Users/Your Username/Folder Name/ExampleMap.csv on Mac." << endl;
   int error;
   do
   {
      error = parseCSV();
   } while (error != 0);

   initializeLists();
   calculatePath();
   vector<int>& path = nodes.back().bestPath;
   int pathLength = path.size();
   vector<double> speeds(pathLength), fuel(pathLength), oxygen(pathLength), food(pathLength);
   if (optimizePath(path, speeds, fuel, oxygen, food) == 0) printData(path, speeds, fuel, oxygen, food);
   else cout << "Invalid path" << endl;

   // Convert microseconds to milliseconds, fix 3 decimal places
   long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
   cout << "Calculation time (milliseconds): " << fixed << setprecision(3) << calcTime / 1000.0 << endl;

   cout << endl;
   while (true) testPath();
}

// Commented-out tests

// Resource drop testing
/*// If resources drop at or below 0, adjust speed
bool adjustedSpeed = false;
if (oxygenLeft <= 0)
{
   if (resupply == Resupply::OXYGEN)
   {
      // If going to oxygen resupply, adjust speed (v) to exhaust oxygen supply
      // Oxygen = currentOxygen - Kl*t = currentOxygen - Kl*d/v = 0.01
      speed = Kl * d / (currentOxygen - 0.01);
   }
   else
   {
      // Exhaust 50% of supply
      speed = 2 * Kl * d / currentOxygen;
   }
   adjustedSpeed = true;
}
if (foodLeft <= 0)
{
   if (resupply == Resupply::FOOD) speed = Kl * d / (currentFood - 0.01);
   else speed = 2 * Kl * d / currentFood;
   adjustedSpeed = true;
}
if (fuelLeft <= 0)
{
   // Adjust speed (v) to exhaust only 50% of remaining fuel
   // Fuel = currentFuel - Kf*d*v^2 = currentFuel/2
   speed = sqrt(currentFuel / (2 * Kf * d));
   adjustedSpeed = true;
}
if (adjustedSpeed)
{
   fuelConsumed = Kf*d*speed*speed, fuelLeft = currentFuel - fuelConsumed;
   t = d/speed;
   oxygenConsumed = foodConsumed = Kl*t;
   oxygenLeft = currentOxygen - oxygenConsumed, foodLeft = currentFood - foodConsumed;
}*/

// Loop testing
/*while (true)//DEBUG
   {
      double x1, y1, x2, y2, x3, y3, r;
      cout << "x1: ";
      cin >> x1;
      cout << "y1: ";
      cin >> y1;
      cout << "x2: ";
      cin >> x2;
      cout << "y2: ";
      cin >> y2;
      cout << "x3: ";
      cin >> x3;
      cout << "y3: ";
      cin >> y3;
      cout << "r: ";
      cin >> r;
      cout << endl;
      testCircle(x1, y1, x2, y2, x3, y3, r);
      cout << endl;
      cout << endl;
   }*/

// Hostile radius testing
/*bool testCircle(double x1, double y1, double x2, double y2, double x3, double y3, double r)
{
   double dx = x1-x3, dy = y1-y3;
   double rise = y2-y1, run = x2-x1, theta = atan2(rise, run);
   double b = 2*dx*cos(theta) + 2*dy*sin(theta), c = dx*dx + dy*dy - r*r;
   double discriminant = b*b - 4*c;
   cout << "disc = " << discriminant << endl;
   if (discriminant < 0) return false;
   double d1 = (-b + sqrt(discriminant)) / 2, d2 = (-b - sqrt(discriminant)) / 2;
   double l = run*run + rise*rise;
   cout << "roots^2: " << d1*d1 << ", " << d2*d1 << ", l=" << l << endl;
   if ((d1 < 0 && d2 < 0) || (d1*d1 > l && d2*d2 > l)) return false;
   cout << "true" << endl;
   return true;
}*/