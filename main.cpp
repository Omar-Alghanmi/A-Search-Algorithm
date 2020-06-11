#include <iostream>
#include "A_Star.h"

using namespace std;
using namespace A_Star;

int main()
{
    Generator generator;
    generator.setWorldSize({25, 25});
    generator.setHeuristic(Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    cout << "Generate path ... \n";
    auto path = generator.findPath({0, 0}, {20, 20});

    for(auto& coordinate : path) {
        cout << coordinate.x << " " << coordinate.y << "\n";
    }
}