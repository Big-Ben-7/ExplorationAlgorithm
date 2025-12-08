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
#define MAX_FUEL 52.75
#define MAX_OXYGEN 16
#define MAX_FOOD 16
#define infinity (numeric_limits<double>::infinity())

using namespace std;

// Use enum instead of string for resupply to improve performance
enum class Resupply { NONE, OXYGEN, FOOD, BOTH };
enum class FunctionType { CURVED, STRAIGHT };
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
// Edges defined by starting node/vertex (planet), ending node, and distance
struct Edge
{
   int node1, node2;
   double distance;
   Edge(int n1, int n2, double d)
   {
      node1 = n1, node2 = n2, distance = d;
   }
   bool operator<(const Edge& other) const
   {
      int other1 = other.node1;
      if (node1 == other1) return node2 < other.node2;
      else return node1 < other1;
   }
};
// Stores cost, resource consumption, and order of travel and speed for best path to node
struct NodeData
{
   double minCost, fuel, food, oxygen;
   vector<int> bestPath;
   NodeData(double initialCost)
   {
      minCost = initialCost;
      fuel = MAX_FUEL;
      oxygen = MAX_OXYGEN;
      food = MAX_FOOD;
      bestPath = vector<int>();
   }
};
vector<vector<double>> nodeDistances;
vector<PlanetData> planets;
int nodeCount;
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
double calculateDistance(const Point pos1, const Point pos2)
{
   double dx = pos2.x - pos1.x;
   double dy = pos2.y - pos1.y;
   return sqrt(dx * dx + dy * dy);
}
// Bisection algorithm to determine roots of function
double bisection(double a, double b, const double x0, const double y0, const double r, double (*func)(const double, const double, const double, const double), const double tolerance = 1e-6)
{
   double c;
   c = 0.5 * (a + b);
   double fa = func(a, x0, y0, r);
   double fb = func(b, x0, y0, r);
   double accuracy = tolerance * 1e-3;
   while (fabs(b - a) > tolerance * c)
   {
      double fc = func(c, x0, y0, r);
      if (fabs(fc) < accuracy) return c;
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
      cout << "Error: Entry 1 (header) must be formatted as \"Planet Name,x,y,type,resupply,hostile_radius\"." << endl;
      return -1;
   }
   int commaIndex;
   string headerSections[] = {"planet name", "x", "y", "type", "resupply", "hostile_radius"};
   for (int i = 0; i < 5; i++)
   {
      commaIndex = line.find(',');
      if(trim(toLower(line.substr(0, commaIndex))) != headerSections[i])
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
      if (trim(line) == "") continue; // Skip empty lines
      lineNumber ++;
      int extraCommas = count(line.begin(), line.end(), ',') - 5;
      if (extraCommas > 0)
      {
         if (extraCommas == 1) cout << "Error: Entry " << lineNumber << " has an extra comma." << endl;
         else cout << "Error: Entry " << lineNumber << " has " << extraCommas << " extra commas." << endl;
         return -1;
      }
      else if (extraCommas < 0)
      {
         extraCommas *= -1;
         if (extraCommas == 1) cout << "Error: Entry " << lineNumber << " needs an extra comma." << endl;
         else cout << "Error: Entry " << lineNumber << " needs " << extraCommas << " extra commas." << endl;
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
      planets.emplace_back(planetName, x, y, resupplyEnum, hostileRadius);
   }
   nodeCount = lineNumber - 1;
   iFile.close();
   return 0;
}

bool edgeInHostileRadius(const int startNode, const int endNode)
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
         double distance = nodeDistances[i][j] = nodeDistances[j][i] = calculateDistance(planets[i].position, planets[j].position);
         // Eliminate edges from target planet for efficiency
         if (i != nodeCount - 1) edgeList.emplace_back(i, j, distance);
         if (j != nodeCount - 1) edgeList.emplace_back(j, i, distance);
      }
   }
   sort(edgeList.begin(), edgeList.end());
}

// Stage 1: Implement modified Bellman-Ford algorithm to find optimal path
void calculatePath()
{
   int edgeCount = edgeList.size();
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
            default:
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
            node2Data.bestPath = node1Data.bestPath;
            node2Data.bestPath.push_back(node2);
            if (!unrelaxed) unrelaxed = true;
         }
      }
   }
}

double derivFunc(const double theta, const double x0, const double y0, const double r)
{
   double cosTheta = cos(theta), sinTheta = sin(theta);
   return x0 * cosTheta - y0 * sinTheta - r * (cosTheta * cosTheta - sinTheta * sinTheta);
}

double func(const double theta, const double x0, const double y0, const double r)
{
   double cosTheta = cos(theta), sinTheta = sin(theta);
   return x0 * sinTheta + y0 * cosTheta - r * sinTheta * cosTheta;
}

int getIdealSpeed(double& v1, double& v2, double& score, const double m, const double c, const double x1, const double y1, const double x2, 
                  const double y2, const double d1, const double d2, const double r, const FunctionType oxygenType, const FunctionType foodType)
{
   double xi1, xi2;
   cout << d1 << "," << d2 << "," << r << endl;
   if (oxygenType == FunctionType::CURVED) xi1 = x1 + y1 / m;
   else
   {
      double m2 = y1 / x1;
      if (m >= m2) xi1 = x1;
      else xi1 = y1 / m;
   }
   if (foodType == FunctionType::CURVED) xi2 = x2 + y2 / m;
   else
   {
      double m2 = y2 / x2;
      if (m >= m2) xi2 = x2;
      else xi2 = y2 / m;
   }
   double innerIntersection = max(xi1, xi2), outerIntersection = r / sqrt(1 + m * m);
   if (innerIntersection >= outerIntersection) return -1;
   double beta = m * sqrt(d1 / d2); // Ratio of velocities
   double v1Ideal = cbrt((d1 * (1 + c) + 1/beta * d2 * (1 + 2 * Kl)) / (2 * (Kf * d1 + beta * beta * Kf * d2)));
   double di = 0.01 * (outerIntersection - innerIntersection);
   double rootd1 = sqrt(d1), v1Min = (innerIntersection+di) / rootd1, v1Max = (outerIntersection-di) / rootd1;
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
   v2 = beta * v1;
   score = d1/v1*(1+c) + Kf*d1*v1*v1 + d2/v2*(1+2*Kl) + Kf*d2*v2*v2;
   printf("%g %g %g %g %g %g %g %g %g\n", x1, y1, x2, y2, r, d1, d2,innerIntersection, outerIntersection);
   cout << m << ", " << v1 << ", " << v2 << ", " << score << ", " << returnVal << endl;
   return returnVal;
}

int optimizeSpeed(double& v1, double& v2, const double d1, const double d2, const Resupply resupplyType, const Resupply resupplyAfter, const double initialFuel, const double initialOxygen, const double initialFood)
{
   int returnVal = -1;
   double rootd1 = sqrt(d1), rootd2 = sqrt(d2);
   double kd1 = Kl * rootd1 * d1, kd2 = Kl * rootd2 * d2;
   double xOxygen, yOxygen, xFood, yFood, r = sqrt(initialFuel / Kf);
   double idealSlope, minSlope, maxSlope, score, c;
   cout << kd1/initialFood << ", " << kd2/initialFood << ", " << r << endl;
   FunctionType oxygenType, foodType;;
   switch (resupplyType)
   {
      case Resupply::OXYGEN:
      {
         xOxygen = kd1 / initialOxygen, yOxygen = kd2 / initialOxygen;
         if (xOxygen * xOxygen + yOxygen * yOxygen >= r * r) return -1;
         xFood = kd1 / initialFood, yFood = kd2 / initialFood;
         if (xFood * xFood + yFood * yFood >= r * r) return -1;
         double thetaMin = bisection(0, M_PI_2, xFood, yFood, r, derivFunc);
         double root1 = bisection(0, thetaMin, xFood, yFood, r, func), root2 = bisection(thetaMin, M_PI_2, xFood, yFood, r, func);
         double theta1 = max(asin(yOxygen/r), root1), theta2 = min(acos(xOxygen/r), root2);
         minSlope = tan(theta1), maxSlope = tan(theta2);
         switch (resupplyAfter)
         {
            case Resupply::FOOD:
            case Resupply::BOTH:
               idealSlope = rootd2 / rootd1 * cbrt((1 + 2 * Kl));
               c = 0;
               break;
            default:
               idealSlope = rootd2 / rootd1 * cbrt((1 + 2 * Kl) / (1 + Kl));
               c = Kl;
               break;
         }
         oxygenType = FunctionType::STRAIGHT, foodType = FunctionType::CURVED;
         break;
      }
      case Resupply::FOOD:
      {
         xOxygen = kd1 / initialOxygen, yOxygen = kd2 / initialOxygen;
         if (xOxygen * xOxygen + yOxygen * yOxygen >= r * r) return -1;
         xFood = kd1 / initialFood, yFood = kd2 / initialFood;
         if (xFood * xFood + yFood * yFood >= r * r) return -1;
         double thetaMin = bisection(0, M_PI_2, xOxygen, yOxygen, r, derivFunc);
         double root1 = bisection(0, thetaMin, xOxygen, yOxygen, r, func), root2 = bisection(thetaMin, M_PI_2, xOxygen, yOxygen, r, func);
         double theta1 = max(asin(yFood/r), root1), theta2 = min(acos(xFood/r), root2);
         minSlope = tan(theta1), maxSlope = tan(theta2);
         switch (resupplyAfter)
         {
            case Resupply::OXYGEN:
            case Resupply::BOTH:
               idealSlope = rootd2 / rootd1 * cbrt((1 + 2 * Kl));
               c = 0;
               break;
            default:
               idealSlope = rootd2 / rootd1 * cbrt((1 + 2 * Kl) / (1 + Kl));
               c = Kl;
               break;
         }
         oxygenType = FunctionType::CURVED, foodType = FunctionType::STRAIGHT;
         break;
      }
      case Resupply::NONE:
      {
         xOxygen = kd1 / initialOxygen, yOxygen = kd2 / initialOxygen;
         if (xOxygen * xOxygen + yOxygen * yOxygen >= r * r) return -1;
         xFood = kd1 / initialFood, yFood = kd2 / initialFood;
         if (xFood * xFood + yFood * yFood >= r * r) return -1;
         double thetaMin = bisection(0, M_PI_2, xFood, yFood, r, derivFunc);
         double root1Food = bisection(0, thetaMin, xFood, yFood, r, func), root2Food = bisection(thetaMin, M_PI_2, xFood, yFood, r, func);
         thetaMin = bisection(0, M_PI_2, xOxygen, yOxygen, r, derivFunc);
         double root1Oxygen = bisection(0, thetaMin, xOxygen, yOxygen, r, func), root2Oxygen = bisection(thetaMin, M_PI_2, xOxygen, yOxygen, r, func);
         double theta1 = max(root1Food, root1Oxygen), theta2 = min(root2Food, root2Oxygen);
         minSlope = tan(theta1), maxSlope = tan(theta2);
         switch (resupplyAfter)
         {
            case Resupply::FOOD:
            case Resupply::OXYGEN:
               idealSlope = rootd2 / rootd1 * cbrt((1 + 2 * Kl) / (1 + Kl));
               c = Kl;
               break;
            case Resupply::BOTH:
               idealSlope = rootd2 / rootd1 * cbrt((1 + 2 * Kl));
               c = 0;
               break;
            default:
               idealSlope = rootd2 / rootd1;
               c = 2 * Kl;
               break;
         }
         oxygenType = FunctionType::CURVED, foodType = FunctionType::CURVED;
         break;
      }
      default: return -1;
   }

   //double idealSpeed = cbrt(0.5 * (1 + c)/Kf);
   double ds = 0.01 * (maxSlope - minSlope);
   minSlope += ds, maxSlope -= ds;
   if (idealSlope < minSlope) idealSlope = minSlope;
   else if (idealSlope > maxSlope) idealSlope = maxSlope;
   printf("slope: %g %g %g %g %g\n", minSlope, maxSlope, idealSlope, rootd1, rootd2);
   returnVal = getIdealSpeed(v1, v2, score, idealSlope, c, xOxygen, yOxygen, xFood, yFood, d1, d2, r, FunctionType::STRAIGHT, FunctionType::CURVED);
   return returnVal;
}

// Stage 2: Optimize resource usage given the known path with 3 methods (which increase in both robustness and computation time)
int optimizePath(vector<int>& path, vector<double>& speeds, vector<double>& fuel, vector<double>& oxygen, vector<double>& food)
{
   int error = 0;
   bool foundOxygenResupply = false, foundFoodResupply = false;
   int pathLength = path.size();
   vector<Resupply> resupplyTypes = vector<Resupply>(pathLength), resupplyAfter = vector<Resupply>(pathLength);
   // Iterate backward to find last oxygen and food resupplies
   // Method 1: set ideal speed by disregard both, one, or none of the Kl terms if node is before both, one or none of the resupplies
   for (int i = pathLength - 1; i >= 0; i--)
   {
      Resupply resupply = resupplyTypes[i] = planets[path[i]].resupply;
      switch (resupply)
      {
         case Resupply::OXYGEN:
            if (!foundOxygenResupply)
            {
               foundOxygenResupply = true;
               if (!foundFoodResupply)
               {
                  resupplyAfter[i] = Resupply::NONE;
                  speeds[i] = singleResupplySpeed;
               }
               else
               {
                  resupplyAfter[i] = Resupply::FOOD;
                  speeds[i] = doubleResupplySpeed;
               }
            }
            else
            {
               if (!foundFoodResupply)
               {
                  resupplyAfter[i] = Resupply::OXYGEN;
                  speeds[i] = singleResupplySpeed;
               }
               else
               {
                  resupplyAfter[i] = Resupply::BOTH;
                  speeds[i] = doubleResupplySpeed;
               }
            }
            break;
         case Resupply::FOOD:
            if (!foundFoodResupply)
            {
               foundFoodResupply = true;
               if (!foundOxygenResupply)
               {
                  resupplyAfter[i] = Resupply::NONE;
                  speeds[i] = singleResupplySpeed;
               }
               else
               {
                  resupplyAfter[i] = Resupply::OXYGEN;
                  speeds[i] = doubleResupplySpeed;
               }
            }
            else
            {
               foundFoodResupply = true;
               if (!foundOxygenResupply)
               {
                  resupplyAfter[i] = Resupply::FOOD;
                  speeds[i] = singleResupplySpeed;
               }
               else
               {
                  resupplyAfter[i] = Resupply::BOTH;
                  speeds[i] = doubleResupplySpeed;
               }
            }
            break;
         default:
            if (foundOxygenResupply && foundFoodResupply)
            {
               resupplyAfter[i] = Resupply::BOTH;
               speeds[i] = doubleResupplySpeed;
            }
            else if (foundOxygenResupply && !foundFoodResupply)
            {
               resupplyAfter[i] = Resupply::OXYGEN;
               speeds[i] = singleResupplySpeed;
            }
            else if (foundFoodResupply && !foundOxygenResupply)
            {
               resupplyAfter[i] = Resupply::FOOD;
               speeds[i] = singleResupplySpeed;
            }
            else
            {
               resupplyAfter[i] = Resupply::NONE;
               speeds[i] = idealSpeed;
            }
            break;
      }
   }
   // It can be mathematically proven (eg. with Lagrange multipliers) that the ideal speed is the same for all nodes with the same resupplyAfter type
   double fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
   double totalDistance = 0;
   int node0 = 0;
   for (int i = 0; i < pathLength; i++)
   {
      int node = path[i];
      double speed = speeds[i], distance = nodeDistances[node0][node], t = distance / speed, consumed = Kl * t;
      Resupply resupply = resupplyTypes[i] = planets[node].resupply;
      switch (resupply)
      {
         case Resupply::OXYGEN:
            foodLeft -= consumed;
            if (error == 0 && (consumed > oxygenLeft - 0.0001 || foodLeft < 0.0001)) error = -1;
            oxygenLeft = MAX_OXYGEN;
            break;
         case Resupply::FOOD:
            oxygenLeft -= consumed;
            if (error == 0 && (consumed > foodLeft - 0.0001 || oxygenLeft < 0.0001)) error = -1;
            foodLeft = MAX_FOOD;
            break;
         default:
            foodLeft -= consumed, oxygenLeft -= consumed;
            if (error == 0 && (oxygenLeft < 0.0001 || foodLeft < 0.0001)) error = -1;
            break;
      }
      fuelLeft -= Kf * distance * speed * speed;
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      totalDistance += distance;
      node0 = node;
   }
   //if (error == 0 && fuelLeft >= 0.0001) return 0;

   // Method 2: If Method 1 (set ideal case) fails, adjust all speeds by constant factor
   double totalFuelConsumed = MAX_FUEL - fuelLeft;
   double speedFactor = sqrt((MAX_FUEL - 0.0001) / (totalFuelConsumed));
   fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
   node0 = error = 0;
   if (totalFuelConsumed > MAX_FUEL - 0.0001)
   {
      for (int i = 0; i < pathLength; i++)
      {
         int node = path[i];
         speeds[i] *= speedFactor;
         double speed = speeds[i], distance = nodeDistances[node0][node], t = distance / speed;
         double consumed = Kl * t, fuelConsumed = Kf * distance * speed * speed;
         fuelLeft -= fuelConsumed;
         if (consumed > foodLeft - 0.0001 || consumed > oxygenLeft - 0.0001 || fuelLeft < 0.0001)
         {
            error = -1;
            break;
         }
         switch (planets[node].resupply)
         {
            case Resupply::OXYGEN:
               oxygenLeft = MAX_OXYGEN, foodLeft -= consumed;
               break;
            case Resupply::FOOD:
               foodLeft = MAX_FOOD, oxygenLeft -= consumed;
               break;
            default:
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
         double minOxygenSpeed = distance / ((oxygenLeft - 0.0001) / Kl);
         double minFoodSpeed = distance / ((foodLeft - 0.0001) / Kl);
         double minSpeed = min(minOxygenSpeed, minFoodSpeed), maxSpeed = speedFactor * speed;
         if (minSpeed > maxSpeed - 0.0001)
         {
            error = -1;
            break;
         }
         if (speed < minSpeed) speeds[i] = speed = minSpeed;
         double consumed = Kl * distance / speed, fuelConsumed = Kf * distance * speed * speed;
         fuelLeft -= fuelConsumed;
         if (consumed > foodLeft - 0.0001 || consumed > oxygenLeft - 0.0001 || fuelLeft < 0.0001)
         {
            error = -1;
            break;
         }
         switch (planets[node].resupply)
         {
            case Resupply::OXYGEN:
               oxygenLeft = MAX_OXYGEN, foodLeft -= consumed;
               break;
            case Resupply::FOOD:
               foodLeft = MAX_FOOD, oxygenLeft -= consumed;
               break;
            default:
               oxygenLeft -= consumed, foodLeft -= consumed;
               break;
         }
         fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
         node0 = node;
      }
   }
   if (error == 0) return 0;

   // Method 3: If Method 2 (constant factor fails), optimize each node's speeds using a combination of algorithmic, analytical, and graphical means
   fuelLeft = MAX_FUEL, oxygenLeft = MAX_OXYGEN, foodLeft = MAX_FOOD;
   double cumulativeDistance = 0;
   node0 = 0;
   error = 0;
   for (int i = 0; i < pathLength; i++)
   {
      int node = path[i];
      double speed, nextSpeed, distance = nodeDistances[node0][node];
      cumulativeDistance += distance;
      if (i < pathLength - 1)
      {
         int rv = optimizeSpeed(speed, nextSpeed, distance, totalDistance - cumulativeDistance, resupplyTypes[i], resupplyAfter[i], fuelLeft, oxygenLeft, foodLeft);
         cout << i << ", "  << distance << ", " << totalDistance-cumulativeDistance << ", " << (int) resupplyTypes[i] << ", " << (int) resupplyAfter[i] << ", " << fuelLeft << ", " << oxygenLeft << ", " << foodLeft << endl;
         if (rv == -1) return -1;
         speeds[i] = speed;
      }
      else speed = speeds[i] = nextSpeed;
      double t = distance / speed, consumed = Kl * t;

      switch (planets[node].resupply)
      {
         case Resupply::OXYGEN:
            foodLeft -= consumed;
            oxygenLeft = MAX_OXYGEN;
            break;
         case Resupply::FOOD:
            oxygenLeft -= consumed;
            foodLeft = MAX_FOOD;
            break;
         default:
            foodLeft -= consumed, oxygenLeft -= consumed;
            if (error == 0 && (oxygenLeft < 0.0001 || foodLeft < 0.0001)) error = -1;
            break;
      }
      fuelLeft -= Kf * distance * speed * speed;
      fuel[i] = fuelLeft, oxygen[i] = oxygenLeft, food[i] = foodLeft;
      node0 = node;
   }
   if (fuelLeft > 0 && oxygenLeft > 0 && foodLeft > 0) return 0;
   else return -1;
}

int oldOptimizePath(vector<int>& path, vector<double>& speeds, vector<double>& fuel, vector<double>& oxygen, vector<double>& food)
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
         default:
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
   double speedFactor = sqrt((MAX_FUEL - 0.0001) / totalFuelConsumed);
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
         if (consumed > foodLeft - 0.0001 || consumed > oxygenLeft - 0.0001) return -1;
         switch (planets[node].resupply)
         {
            case Resupply::OXYGEN:
               oxygenLeft = MAX_OXYGEN, foodLeft -= consumed;
               break;
            case Resupply::FOOD:
               foodLeft = MAX_FOOD, oxygenLeft -= consumed;
               break;
            default:
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
         double minOxygenSpeed = distance / ((oxygenLeft - 0.0001) / Kl);
         double minFoodSpeed = distance / ((foodLeft - 0.0001) / Kl);
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
            default:
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
   cout << "Planet travel order (all values rounded to 4 decimals and in AU): " << endl;
   cout << endl;
   cout << " " << right << setw(14) << "Planet: " << setw(14) << "Speed: " << setw(14) << "Dist: " << setw(14) << "Total Time: " << setw(14) << "Fuel: " << setw(14) << "Oxyg: " << setw(14) << "Food: " << setw(14) << "Score: " << endl;
   cout << fixed;
   int pathLength = path.size();
   int node0 = 0;
   double travelTime = 0;
   for (int i = 0; i < pathLength; i++)
   {
      int node = path[i];
      double d = nodeDistances[node0][node], v = speeds[i], t = d/v;
      travelTime += t;
      double fu = fuel[i], o = oxygen[i], fo = food[i], s = travelTime - (fu + o + fo);
      if (edgeInHostileRadius(node0, node)) cout << "DEBUG: Edge crosses hostile radius" << endl;
      cout << i + 1 << ") " 
           << right
           << setw(11) << planets[node].name << "   " 
           << setw(11) << setprecision(9) << v << "   " 
           << setw(11) << setprecision(6) << d << "   " 
           << setw(11) << travelTime << "   " 
           << setw(11) << fu << "   " 
           << setw(11) << o << "   " 
           << setw(11) << fo << "   " 
           << setw(11) << s
           << endl;
      node0 = node;
   }
   cout << endl;
   cout << "Final stats: " << endl;
   double fuelLeft = fuel.back(), oxygenLeft = oxygen.back(), foodLeft = food.back();
   cout << "Total time: " << travelTime << endl;
   cout << "Remaining fuel: " << fuelLeft << endl;
   cout << "Remaining oxygen: " << oxygenLeft << endl;
   cout << "Remaining food: " << foodLeft << endl;
   cout << "Score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

void testPath()
{
   cout << endl;
   string input;
   cout << "Path (New): ";
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

void oldTestPath()
{
   cout << endl;
   string input, input2;
   cout << "Path (Old): ";
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
         if (planets[i].name != planet) continue;
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
      if (planets[i].name != planet) continue;
      path.push_back(i);
      break;
   }

   cout << endl;
   cout << "Map used: " << mapName << endl;
   cout << "Planet travel order (all values rounded to 4 decimals and in AU): " << endl;
   cout << endl;
   cout << " " << right << setw(14) << "Planet: " << setw(14) << "Speed: " << setw(14) << "Dist: " << setw(14) << "Travel Time: " << setw(14) << "Fuel: " << setw(14) << "Oxyg: " << setw(14) << "Food: " << setw(14) << "Score: " << endl;
   cout << fixed;
   int pathLength = path.size();
   vector<array<double, 3>> pathData;
   int node0 = 0;
   double travelTime = 0;
   for (int i = 0; i < pathLength; i++)
   {
      int node = path[i];
      array<double, 3> previousNode;
      if (i != 0) previousNode = pathData[i-1];
      else previousNode = {MAX_FUEL, MAX_OXYGEN, MAX_FOOD};
      double v = speeds[i], t, o, fo, d = calculateDistance(planets[node0].position, planets[node].position);
      switch (planets[node].resupply)
      {
         case Resupply::OXYGEN:
            t = d/v;
            o = MAX_OXYGEN;
            fo = previousNode[2] - Kl * t;
            break;
         case Resupply::FOOD:
            t = d/v;
            fo = MAX_FOOD;
            o = previousNode[1] - Kl * t;
            break;
         default:
            t = d/v;
            o = previousNode[1] - Kl * t, fo = previousNode[2] - Kl * t;
            break;
      }
      double fu = previousNode[0] - Kf * d * v * v;
      travelTime += t;
      double s = travelTime-(fu+o+fo);
      pathData.push_back({fu, o, fo});
      if (edgeInHostileRadius(node0, node)) cout << "DEBUG: Edge crosses hostile radius" << endl;
      cout << i + 1 << ") " 
           << right
           << setw(11) << planets[node].name << "   " 
           << setw(11) << setprecision(9) << v << "   " 
           << setw(11) << setprecision(6) << d << "   " 
           << setw(11) << travelTime << "   " 
           << setw(11) << fu << "   " 
           << setw(11) << o << "   " 
           << setw(11) << fo << "   " 
           << setw(11) << s
           << endl;
      node0 = node;
   }
   cout << endl;
   cout << "Final stats: " << endl;
   array<double, 3> lastNode = pathData[pathLength - 1];
   double fuelLeft = lastNode[0], oxygenLeft = lastNode[1], foodLeft = lastNode[2];
   cout << "Travel time: " << travelTime << endl;
   cout << "Remaining fuel: " << fuelLeft << endl;
   cout << "Remaining oxygen: " << oxygenLeft << endl;
   cout << "Remaining food: " << foodLeft << endl;
   cout << "Final score: " << travelTime - (fuelLeft + oxygenLeft + foodLeft) << endl;
}

int main()
{
   cout << endl;
   cout << "Welcome Mission Control!" << endl;
   cout << endl;
   cout << "Use these instructions to begin stress-testing:" << endl;
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
   cout << "New: " << endl;
   if (optimizePath(path, speeds, fuel, oxygen, food) == 0) printData(path, speeds, fuel, oxygen, food);
   cout << endl;
   cout << "Old: " << endl;
   if (oldOptimizePath(path, speeds, fuel, oxygen, food) == 0) printData(path, speeds, fuel, oxygen, food);
   else cout << "Invalid path" << endl;

   // Convert microseconds to seconds, fix 6 decimal places
   long long calcTime = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - startTime).count();
   cout << "Calculation time (seconds): " << calcTime / 1000000.0 << endl;

   cout << endl;
   while (true)
   {
      oldTestPath();
      testPath();
   }
}