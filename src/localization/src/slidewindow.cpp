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

    double update(double newX) {
        // Add the new value to the buffer
        bufferX.push_back(newX);
        sumX += newX;

        // If the buffer size exceeds the window size, remove the oldest value
        if (bufferX.size() > windowSize) {
            sumX -= bufferX.front();
            bufferX.erase(bufferX.begin());

            // Calculate and return the moving average for x
            double avgX = sumX / bufferX.size();
            return avgX;
        } else {
            return newX;
        }
    }
};
