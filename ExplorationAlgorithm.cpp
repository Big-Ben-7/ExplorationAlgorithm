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

using namespace std;

enum class Resupply { NONE, OXYGEN, FOOD };
struct Point
{
   double x, y;
};
struct Planet
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
vector<Planet> planets;
int nodeCount, edgeCount;
chrono::steady_clock::time_point startTime;
// Edge list used to efficiently represent graph
vector<Edge> edgeList;
vector<NodeData> nodes;

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
   string fileName = "ExampleMap.csv", line = "";
   /*cout << "Map file name: ";
   cin >> fileName;*/
   // Start computation time immediately after user enters file name.
   startTime = chrono::steady_clock::now();
   iFile.open(fileName);
   if (!iFile.is_open())
   {
      cout << "Error: file not found. Please make sure that the entered name is correct and that the CSV file is in the same folder as this project." << endl;
      return -1;
   }

   // Header line error handling
   getline(iFile, line);
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
      // Catch incorrect CSV formatting
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
   Point start = planets[startNode].position, end = planets[endNode].position;
   // Line eq. (parametric): x = x1 + cos(theta)*d, y = y1 + sin(theta)*d where d=distance along line and theta = atan(slope)
   double x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y, rise = y2-y1, run = x2-x1;
   double theta = atan2(rise, run), cosTheta = cos(theta), sinTheta = sin(theta); // Precompute trig for efficiency
   double l = run*run + rise*rise; // Length of line
   for (int i = 0; i < nodeCount; i++)
   {
      if (planets[i].hostileRadius == 0) continue;
      Point center = planets[i].position;
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
      //cout << startNode << ", " << endNode << " TRUEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE" << endl; //DEBUG
      return true;
   }
   return false;
}

void initializeEdgeList()
{
   // Set distance to initial node to 0
   nodes.push_back(NodeData(0));
   for (int i = 0; i < nodeCount; i++)
   {
      if (i != 0)
      {
         // Set pre-relaxed distance to all other nodes to infinity
         nodes.push_back(NodeData(numeric_limits<double>::infinity()));
      }
      if (planets[i].hostileRadius != 0) continue;
      // Fill edge list with edges for each pair of connected nodes
      for (int j = 0; j < nodeCount; j++)
      {
         // Each planet connected to all other non-hostile planets
         if (j == i || edgeInHostileRadius(i, j)) continue;
         edgeList.push_back({i, j, calculateDistance(planets[i].position, planets[j].position)});
         //cout << i << ", " << j << endl; //DEBUG
      }
   }
   edgeCount = edgeList.size();
   //cout << endl; //DEBUG
}

// Implemented modified Bellman-Ford algorithm for negative weights due to refills
void calculatePath()
{
   // Speed optimization by first derivative test
   // Cost = c(x) = time + fuel + food + oxygen = d/v + Kf*d*v^2 + 2*Kl*d/v
   // c'(x) = -d/v^2 + 2*Kf*d*v + -2*Kl*d/v^2 = (-d + 2*Kf*d*v^3 - 2*Kl*d)/v^2
   // Cost minimized at c'(x) = 0 => -d + 2*Kf*d*v^3 - 2*Kl*d = 0 => v = cbrt((1 + 2*Kl) / (2*Kf))
   double idealSpeed = cbrt((1 + 2 * Kl) / (2 * Kf));

   bool unrelaxed = true;
   while (unrelaxed)
   {
      // Terminate once one iteration has passed with no relaxations
      // (Any negative cycles due to refills will vanish over iterations as food/oxygen increase)
      unrelaxed = false;
      for (int i = 0; i < edgeCount; i++)
      {
         Edge* edge = &edgeList[i];
         int node1 = edge->node1, node2 = edge->node2;
         NodeData* node1Data = &nodes[node1];
         NodeData* node2Data = &nodes[node2];
         Resupply resupply = planets[node2].resupply;

         double d = edge->distance, speed = idealSpeed; //newSpeed1 = 0, newSpeed2 = 0, newSpeed3 = 0;
         double currentFuel = node1Data->fuel, currentOxygen = node1Data->oxygen, currentFood = node1Data->food;
         double fuelConsumed = Kf*d*speed*speed, fuelLeft = currentFuel - fuelConsumed;
         double oxygenConsumed, foodConsumed, t = d/speed;
         oxygenConsumed = foodConsumed = Kl*t;
         double oxygenLeft = currentOxygen - oxygenConsumed, foodLeft = currentFood - foodConsumed;
         // If resources drop at or below 0, adjust speed
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
         }
         // Dynamically assign weights (travel cost) based on time, resource consumption, and refills
         // Refills can cause negative weights
         switch (resupply)
         {
            case Resupply::OXYGEN:
               oxygenConsumed = currentOxygen - MAX_OXYGEN;
               oxygenLeft = MAX_OXYGEN;
               break;
            case Resupply::FOOD:
               foodConsumed = currentFood - MAX_FOOD;
               foodLeft = MAX_FOOD;
               break;
            case Resupply::NONE:
               break;
         }
         double newCost = node1Data->minCost + t + fuelConsumed + oxygenConsumed + foodConsumed;

         //cout << node1 << ", " << node2 << " checked" << endl; //DEBUG
         if (newCost < node2Data->minCost)
         {
            // Relax edge
            node2Data->minCost = newCost;
            node2Data->fuel = fuelLeft;
            node2Data->food = foodLeft;
            node2Data->oxygen = oxygenLeft;
            node2Data->time = node1Data->time + t;
            node2Data->speeds = node1Data->speeds;
            node2Data->speeds.push_back(speed);
            node2Data->bestPath = node1Data->bestPath;
            node2Data->bestPath.push_back(node2);
            if (!unrelaxed) unrelaxed = true;
            //cout << i << ", " << node << " relaxed" << endl; //DEBUG
         }
      }
   }
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

   initializeEdgeList();
   calculatePath();
   
   NodeData lastNode = nodes[nodeCount - 1];
   vector<double> speeds = lastNode.speeds;
   vector<int> path = lastNode.bestPath;
   int pathLength = path.size();
   cout << "Planet travel order (planet, speed, distance, time, fuel, oxygen, food, score):" << endl;
   //try
   //{
   for (int i = 0; i < pathLength; i++)
   {
      int node = path[i];
      double d = calculateDistance((i == 0) ? Point({0, 0}) : planets[node-1].position, planets[node].position), t = d/speeds[i];
      double fu = nodes[node].fuel, o = nodes[node].oxygen, fo = nodes[node].food, sc = t-(fu+o+fo);
      cout << i+1 << ") " 
           << left
           << setw(10) << planets[node].name + ", " 
           << setw(10) << speeds[i] << ", " 
           << setw(10) << d << ", " 
           << setw(10) << t << ",  " 
           << setw(10) << fu << ", " 
           << setw(10) << o << ", " 
           << setw(10) << fo << ", " 
           << setw(10) << sc 
           << endl;
   }
   cout << endl;
   double travelTime = lastNode.time, fuelLeft = lastNode.fuel, oxygenLeft = lastNode.oxygen, foodLeft = lastNode.food;
   cout << "Travel time (time-AU): " << travelTime << endl;
   cout << "Remaining fuel: " << fuelLeft << endl;
   cout << "Remaining oxygen: " << oxygenLeft << endl;
   cout << "Remaining food: " << foodLeft << endl;
   cout << "Score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
   //}
   //catch (...)
   //{
   //   cout << "ERROR" << endl;
   //}

   long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
   // Convert microseconds to milliseconds, fix 3 decimals
   cout << endl;
   cout << "Calculation time (milliseconds): " << fixed << setprecision(3) << calcTime / 1000.0 << endl;

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
}

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