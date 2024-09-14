#ifndef VALVE_CONTROLLER_HPP
#define VALVE_CONTROLLER_HPP

#include <vector>
#include <array>

enum class ValveType {
    FUEL,
    OXIDIZER,
    MAIN,
    PURGE
};

struct ValveState {
    bool isOpen;
    double openTime;
    double closeTime;
};

class ValveController {
public:
    explicit ValveController(size_t numTubes);
    ~ValveController() = default;

    void initialize();
    void updateValves(double currentTime);
    void openValve(size_t tubeIndex, ValveType type);
    void closeValve(size_t tubeIndex, ValveType type);
    ValveState getValveState(size_t tubeIndex, ValveType type) const;
    void setValveTiming(size_t tubeIndex, ValveType type, double openTime, double closeTime);

private:
    size_t m_numTubes;
    std::vector<std::array<ValveState, 2>> m_valveStates;  // [tubeIndex][ValveType]
    double m_currentTime;

    void validateTubeIndex(size_t tubeIndex) const;
};

#endif // VALVE_CONTROLLER_HPP