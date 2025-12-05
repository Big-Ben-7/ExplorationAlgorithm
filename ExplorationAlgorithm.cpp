#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#define kf 0.15
#define kl 0.3

using namespace std;

struct Point
{
   double x;
   double y;
};
vector<string> planetNames, resupplies;
vector<Point> positions;
vector<double> hostileRadii;
int vertexCount;
chrono::steady_clock::time_point startTime;

// Adjacency list to efficiently represent graph
// List of all vertices (planets) connected to each vertex in form pair<connected vertex index, edge weight>
vector<vector<pair<int, double>>> adjacencyList;
// Distances of shortest path to each vertex
vector<double> minDistances;
// List of vertex indices making up shortest path to each vertex
vector<vector<int>> paths;

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
      if (resupply != "" && resupply != "none" && resupply != "oxygen" && resupply != "food")
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
      
      // Initialize
      planetNames.push_back(planetName);
      positions.push_back({x, y});
      resupplies.push_back(resupply);
      hostileRadii.push_back(hostileRadius);
   }
   vertexCount = lineNumber - 1;
   iFile.close();
   return 0;
}

bool edgeInHostileRadius(int startVertex, int endVertex)
{
   //cout << endl;
   // Calculate intersection of line through (x1,y1) and (y1,y2) with circle at (x3,y3) and radius r
   Point start = positions[startVertex], end = positions[endVertex];
   // Line eq. (parametric): x = x1 + cos(theta)*d, y = y1 + sin(theta)*d where d=distance along line and theta = atan(slope)
   double x1 = start.x, y1 = start.y, x2 = end.x, y2 = end.y;
   double rise = y2-y1, run = x2-x1, theta = atan2(rise, run);
   double l = run*run + rise*rise;
   for (int i = 0; i < vertexCount; i++)
   {
      if (hostileRadii[i] == 0) continue;
      Point center = positions[i];
      // Circle eq.: (x-x3)^2 + (y-y3)^2 = r^2
      double x3 = center.x, y3 = center.y, r = hostileRadii[i], dx = x1-x3, dy = y1-y3;
      // Plugging line eqs. into circle eq. and applying trig identities yields quadratic a*d^2 + b*d + c = 0
      double b = 2*dx*cos(theta) + 2*dy*sin(theta), c = dx*dx + dy*dy - r*r; // a=1
      double discriminant = b*b - 4*c;
      // If discriminant < 0, no solution and edge does not cross hostile circle
      if (discriminant < 0) continue;
      double d1 = (-b + sqrt(discriminant)) / 2, d2 = (-b - sqrt(discriminant)) / 2; // Roots from quadratic formula
      // If both values for d are lower or higher than edge length, hostile circle is still not crossed
      if ((d1 < 0 && d2 < 0) || (d1*d1 > l && d2*d2 > l)) continue;
      //cout << startVertex << ", " << endVertex << " TRUEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE" << endl; //DEBUG
      return true;
   }
   return false;
}

void setAdjacencyList()
{
   // Set initial vertex distance to 0 and all others to infinity
   minDistances.push_back(0);
   for (int i = 0; i < vertexCount; i++)
   {
      if (i != 0) minDistances.push_back(numeric_limits<double>::infinity());
      paths.push_back(vector<int>());
      adjacencyList.push_back(vector<pair<int, double>>());
      if (hostileRadii[i] != 0) continue;
      // Fill adjacency list with each vertex's connected vertices and weights (cost)
      for (int j = 0; j < vertexCount; j++)
      {
         // Each planet connected to all other non-hostile planets
         if (j == i || edgeInHostileRadius(i, j)) continue;
         // Use distance consumed as initial edge weights, add in fuel, oxygen, food, time dynamically based on optimized speed
         adjacencyList[i].push_back({j, calculateDistance(positions[i], positions[j])});
         //cout << i << ", " << j << endl; //DEBUG
      }
   }
   //cout << endl; //DEBUG
}

// Implemented modified Bellman-Ford algorithm for negative weights due to refills
void calculatePath()
{
   // First derivative test speed optimization
   // Cost = c(x) = time + fuel + food + oxygen = d/v + kf*d*v^2 + 2*kl*d/v
   // c'(x) = -d/v^2 + 2*kf*d*v + -2*kl*d/v^2 = (-d + 2*kf*d*v^3 - 2*kl*d)/v^2
   // Cost minimized at c'(x) = 0 => -d + 2*kf*d*v^3 - 2*kl*d = 0 => v = cbrt((1 + 2*kl) / (2*kf))
   //double optimalSpeed = cbrt((1 + 2 * kl) / (2 * kf));
   bool unrelaxed = true;
   while (unrelaxed)
   {
      // Terminate once one iteration has passed with no relaxations
      // (Any negative cycles due to refills will vanish over iterations as food/oxygen increase)
      unrelaxed = false;
      for (int i = 0; i < vertexCount; i++)
      {
         double minDistance = minDistances[i];
         vector<pair<int, double>> connectedVertices = adjacencyList[i];
         int edgeCount = connectedVertices.size();
         for (int j = 0; j < edgeCount; j++)
         {
            pair<int, double> connectedVertex = connectedVertices[j];
            double newDistance = minDistance + connectedVertex.second;
            int vertex = connectedVertex.first;
            //cout << i << ", " << vertex << " checked" << endl; //DEBUG
            if (newDistance < minDistances[connectedVertex.first])
            {
               // Relax edge
               minDistances[vertex] = newDistance;
               paths[vertex] = paths[i];
               paths[vertex].push_back(vertex);
               if (!unrelaxed) unrelaxed = true;
               //cout << i << ", " << vertex << " relaxed" << endl; //DEBUG
            }
         }
      }
   }
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

   setAdjacencyList();
   calculatePath();
   
   vector<int> path = paths[vertexCount - 1];
   int pathLength = path.size();
   string planetOrder = "Planet travel order: " + planetNames[path[0]];
   for (int i = 1; i < pathLength; i++)
   {
      planetOrder += ", " + planetNames[path[i]];
   }
   cout << planetOrder << endl;

   long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
   cout << "Calculation time (milliseconds): " << calcTime / 1000.0 << endl;

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