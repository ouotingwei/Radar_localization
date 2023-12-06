#include <iostream>
#include <vector>

class MovingAverageFilter {
private:
    std::vector<double> bufferX;
    size_t windowSize;
    double sumX;

public:
    MovingAverageFilter(size_t initialWindowSize = 2) : windowSize(initialWindowSize), sumX(0.0) {
        bufferX.reserve(windowSize);
    }

    double update(double newValue) {
    // Add the new value to the buffer
    bufferX.push_back(newValue);
    sumX += newValue;

    // If the buffer size is below a certain threshold, return the raw value
    if (bufferX.size() < windowSize) {
        return newValue;
    }

    // If the buffer size exceeds the window size, remove the oldest value
    sumX -= bufferX.front();
    bufferX.erase(bufferX.begin());

    // Calculate and return the moving average
    return sumX / bufferX.size();
    }

};
