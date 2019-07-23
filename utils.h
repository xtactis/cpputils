#ifndef UTILS
#define UTILS

#include <string>
#include <vector>
#include <tuple>
#include <cmath>
#include <unordered_map>
#include <map>
#include <set>
#include <list>
#include <iostream>
#include <sstream>

class Point {
public:
    Point(double x, double y) : data({x, y}) {}
    Point(const Point &p) : data({p.x(), p.y()}) {}
    Point() : data({0, 0}) {}
    ~Point() = default;

    Point &operator=(const Point &rhs) {
        if (&rhs == this) {
            return *this;
        }
        this->rx() = rhs.x();
        this->ry() = rhs.y();
        return *this;
    }

    Point &operator-=(const Point &rhs) {
        this->rx() -= rhs.x();
        this->ry() -= rhs.y();
        return *this;
    }

    Point &operator+=(const Point &rhs) {
        this->rx() += rhs.x();
        this->ry() += rhs.y();
        return *this;
    }

    Point &operator*=(double factor) {
        this->rx() *= factor;
        this->ry() *= factor;
        return *this;
    }

    Point &operator/=(double factor) {
        this->rx() /= factor;
        this->ry() /= factor;
        return *this;
    }

    friend Point operator-(Point lhs, const Point &rhs) {
        lhs -= rhs;
        return lhs;
    }

    friend Point operator+(Point lhs, const Point &rhs) {
        lhs += rhs;
        return lhs;
    }

    friend Point operator*(Point lhs, double factor) {
        lhs *= factor;
        return lhs;
    }

    friend Point operator*(double factor, Point rhs) {
        return rhs*factor;
    }

    friend Point operator/(Point lhs, double factor) {
        lhs /= factor;
        return lhs;
    }

    friend Point operator/(double factor, Point rhs) {
        return rhs/factor;
    }

    double &rx() { return data[0]; }
    double x() const { return data[0]; }
    double &ry() { return data[1]; }
    double y() const { return data[1]; }

    inline static double dotProduct(const Point &a, const Point &b) {
        return a.x()*b.x()+a.y()*b.y();
    }
private:
    double data[2];
};



class Help {
public:
    Help() : maxParamNames(0), cachedHelpMsg(""), changes(false) {
        parameterNames.emplace_back();
    }
    ~Help() = default;

    void addArgument(const utils::Argument &parameter) {
        *parameterNames.rbegin() += parameter;
        *parameterNames.rbegin() += " ";
        maxParamNames = std::max(maxParamNames, (*parameterNames.rbegin()).size());
        changes = true;
    }

    void setDescription(const std::string &description) {
        descriptions.push_back(description);
        parameterNames.emplace_back();
        changes = true;
    }

    std::string createHelpMessage() const {
        if (!changes) {
            return cachedHelpMsg;
        }
        std::stringstream helpStream;
        const size_t indentation = maxParamNames + (maxParamNames&1) + 2;
        helpStream.str("");
        for (size_t i = 0; i < descriptions.size(); ++i) {
            helpStream << parameterNames[i];
            for (size_t j = parameterNames[i].size(); j < indentation; ++j) {
                helpStream << " ";
            }
            helpStream << indentNewlines(descriptions[i], indentation) << "\n";
        }
        cachedHelpMsg = helpStream.str();
        changes = false;
        return cachedHelpMsg;
    }

private:
    std::vector<std::string> parameterNames;
    std::vector<std::string> descriptions;
    size_t maxParamNames;
    mutable std::string cachedHelpMsg;
    mutable bool changes;

    std::string indentNewlines(const std::string &description, const int indentation) const {
        std::string ret;
        for (const auto &e : description) {
            ret.push_back(e);
            if (e != '\n') continue;
            for (size_t i = 0; i < indentation+2; ++i) {
                ret.push_back(' ');
            }
        }
        return ret;
    }
} helpMsg;

std::ostream &operator<<(std::ostream &os, const Help &helpMsg) { 
    return os << helpMsg.createHelpMessage();
}

namespace utils {
    enum RetCodes : uint32_t {
        OK_NOUSE = 0, // everything went fine, didn't use the parameter you gave me
        OK_USED = 1, // everything went fine, used the parameter
        ERR = 2, // OOPS
        NOTIMPL_NOUSE = 3, // not implemented, doesn't expect a parameter
        NOTIMPL_USED = 4 // not implemented, expects a parameter
    };

    typedef const std::string & Parameter;
    typedef std::function<RetCodes(Parameter)> ResponseFunction;
    typedef std::string Argument;
    typedef std::unordered_map<Argument, ResponseFunction> ResponseLUT;
    typedef std::vector<std::string> ArgumentList;

    typedef std::vector<Point> Polygon;

    double interpolate(double val, double y0, double x0, double y1, double x1);
    double base(double val);
    double red(double gray);
    double green(double gray);
    double blue(double gray);
    double distanceSquared(const Point &q, const Point &p);
    double distance(const Point &q, const Point &p);
    double pointToSegmentSquared(const Point &p, const Point &v, const Point &w);
    double pointToSegment(const Point &p, const Point &v, const Point &w);

    Point findCentroid(const Polygon &polygon);

    ResponseLUT responses;

    void addResponse(const ResponseFunction &, const std::string &description) {
        helpMsg.setDescription(description);
    }

    template<typename ...Args>
    void addResponse(const ResponseFunction &func, const Argument &first, Args... rest) {
        responses[first] = func;
        helpMsg.addArgument(first);
        addResponse(func, rest...);
    }

    void createResponses();

    bool isNumeric(const std::string &s);

    /*
     * Returns a new std::vector with only the unique elements. Preserves order.
     *
     * Complexity: O(nlogn*m)
     * - where n is the size of the vector and m is the number of repeated elements
     */
    template<template <typename, typename ...> class C, typename T, typename ... Ts>
    C<T, Ts...> unsortedUnique(const C<T, Ts...> &v) {
        C<T> ret(v);
        std::set<T> seen;
        uint32_t removed = 0;
        for (size_t i = 0; i < v.size(); ++i) {
            if (!seen.insert(v[i]).second) {
                ret.erase(ret.begin()+i-removed);
                ++removed;
            }
        }
        return ret;
    }

    bool stopExec;
    bool gui;
    int createPedestrians;
    ArgumentList args;
    std::unordered_map<std::string, RetCodes> rets;
    uint32_t parameterCount;

    const bool UNIQUE_ARGS = true;
    const bool NO_UNIQUE_ARGS = false;

    ArgumentList preprocessArgs(int argc, const char * const * const argv, bool uniqueArgs);
    bool parseCmdArgs(int argc, const char * const * const argv, bool uniqueArgs = UNIQUE_ARGS);

    template<typename T>
    T clamp(T x, T lo, T hi) {
        #if __cplusplus > 201402L
            return std::max(lo, std::min(hi, x));
        #else
            return std::clamp(x, lo, hi);
        #endif
    }
};

#endif // UTILS
