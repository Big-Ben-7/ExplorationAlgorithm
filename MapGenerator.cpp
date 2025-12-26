#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>
using namespace std;

int main()
{
    cout << endl;
    string input;
    cout << "Planet number: ";
    getline(cin, input);
    int planetNumber = stoi(input);
    cout << "Max pos: ";
    getline(cin, input);
    double maxPos = stod(input);
    cout << "Max hostile radius: ";
    getline(cin, input);
    double maxRad = stod(input);
    cout << "Hostile probability: ";
    getline(cin, input);
    double hosProb = stod(input);

    random_device rd;
    mt19937 engine(rd());
    uniform_real_distribution<double> xPos(-maxPos, maxPos);
    uniform_real_distribution<double> yPos(-maxPos, maxPos);
    uniform_int_distribution<int> res(0, 2);
    uniform_real_distribution<double> hos(0, 1);
    uniform_real_distribution<double> rad(0.1, maxRad);

    ofstream outFile("RandomMap.csv");
    if (!outFile.is_open())
        return -1;
    outFile << "Planet Name,x,y,type,resupply,hostile_radius" << endl;
    for (int i = 0; i < planetNumber; i++)
    {
        double x = xPos(engine), y = yPos(engine);
        if (i == 0)
            x = y = -maxPos;
        if (i == planetNumber - 1)
            x = y = maxPos;

        int rand = res(engine);
        string resupply = "";
        if (i != 0 && i != planetNumber - 1)
        {
            if (rand == 1)
                resupply = "Oxygen";
            else if (rand == 2)
                resupply = "Food";
        }

        string hosType;
        double hosRad;
        if (hos(engine) <= hosProb && i != 0 && i != planetNumber - 1)
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
}