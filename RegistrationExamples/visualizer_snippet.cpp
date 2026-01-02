#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>
#include <iostream>

int main () {
    Zivid::Visualization::Visualizer v;

    v.showMaximized();

    std::cout << "Running visualizer. Blocking until window closes." << std::endl;
    v.run();

    return 0;   
}