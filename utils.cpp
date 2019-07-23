#include "utils.h"

#include <iostream>
#include <memory>

double utils::interpolate(const double val, const double y0, const double x0, const double y1, const double x1) {
    return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

double utils::base(double val) {
    val = (val+1)/2;
    if (val <= 0.125) return 0;
    if (val <= 0.375) return interpolate(val, 0.5, 0.125, 1.0, 0.375);
    if (val <= 0.625) return 1.0;
    if (val <= 0.875) return interpolate(val, 1.0, 0.625, 0.5, 0.875);
    return 0.0;
}

double utils::red(const double gray) {
    return base(gray - 0.5);
}

double utils::green(const double gray) {
    return base(gray);
}

double utils::blue(const double gray) {
    return base(gray + 0.5);
}

double utils::distanceSquared(const Point &q, const Point &p) {
    return pow(q.x()-p.x(), 2)+pow(q.y()-p.y(), 2);
}

double utils::distance(const Point &q, const Point &p) {
    return sqrt(distanceSquared(q, p));
}

double utils::pointToSegmentSquared(const Point &p, const Point &v, const Point &w) {
    const auto lengthSquared = distanceSquared(v, w);
    if (lengthSquared == 0.0) {
        return distanceSquared(v, p);
    }

    const auto t = utils::clamp(Point::dotProduct(p - v, w - v) / (lengthSquared), 0.0, 1.0);
    const auto projection = v + t * (w - v);

    return distanceSquared(projection, p);
}

double utils::pointToSegment(const Point &p, const Point &v, const Point &w) {
    return sqrt(pointToSegmentSquared(p, v, w));
}

Point utils::findCentroid(const std::vector<Point> &polygon) {
    if (polygon.size() < 2) {
        return polygon[0];
    }
    Point centroid {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    for (unsigned long i = 0, j = polygon.size()-1; i < polygon.size(); j = i++) {
        x0 = polygon[j].x();
        y0 = polygon[j].y();
        x1 = polygon[i].x();
        y1 = polygon[i].y();
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.rx() += (x0 + x1)*a;
        centroid.ry() += (y0 + y1)*a;
    }

    signedArea *= 0.5;
    centroid.rx() /= (6.0*signedArea);
    centroid.ry() /= (6.0*signedArea);

    return centroid;
}

bool utils::isNumeric(const std::string &s) {
    if (s.empty()) return false;
    if (s[0] != '-' && !std::isdigit(s[0])) return false;
    bool decimal = false;
    for (size_t i = 1; i < s.size(); ++i) {
        if (!std::isdigit(s[i])) {
            if (s[i] == '.') {
                if (decimal) {
                    return false;
                } else {
                    decimal = true;
                }
            } else {
                return false;
            }
        }
    }
    return true;
}

// statics

utils::ResponseLUT utils::responses;
bool utils::gui = true;
bool utils::stopExec = false;
int utils::createPedestrians = 0;
uint32_t utils::parameterCount = 0;
utils::ArgumentList utils::args;
std::unordered_map<std::string, utils::RetCodes> utils::rets;

utils::ArgumentList utils::preprocessArgs(int argc, const char * const * const argv, bool uniqueArgs = UNIQUE_ARGS) {
    std::vector<std::string> ret;
    ret.reserve(static_cast<size_t>(argc));

    for (int i = 1; i < argc; ++i) {
        const std::string cur = argv[i];
        if (cur[0] == '-' && cur[1] != '-') {
            for (size_t j = 1; j < cur.size(); ++j) {
                ret.emplace_back("-");
                ret.back() += cur[j];
            }
        } else {
            ret.emplace_back(cur);
        }
    }

    return uniqueArgs ? unsortedUnique(ret) : ret;
}

bool utils::parseCmdArgs(const utils::ArgumentList &args) {
    RetCodes retValue;
    for (size_t i = 0; i < args.size(); ++i) {
        std::string parameter = "";
        if (responses.find(args[i]) == responses.end()) {
            std::cerr << "Unrecognized commandline parameter " << args[i] << std::endl;
            return true; // stop execution
        }
        if (i+1 < args.size()) {
            parameter = args[i+1];
        }
        retValue = responses[args[i]](parameter);
        if (retValue == RetCodes::ERR) {
            return true; // stop execution
        }
        if (retValue == RetCodes::NOTIMPL_NOUSE || retValue == RetCodes::NOTIMPL_USED) {
            std::cerr << "Sorry, " << args[i] << " is not implemented yet!" << std::endl;
            retValue = retValue == RetCodes::NOTIMPL_NOUSE ? RetCodes::OK_NOUSE : RetCodes::OK_USED;
            return true; // stop execution
        }
        if (stopExec) {
            return true; // stop execution
        }
        rets[args[i]] = retValue;
        i += retValue;
        ++parameterCount;
    }

    return false; // all good, don't stop execution
}

void utils::createResponses() {
    addResponse([](Parameter) {
        std::cout << helpMsg << std::endl;
        stopExec = true;
        return utils::OK_NOUSE;
    }, "-h", "--help", "\t\t\tdisplays this help message");
}
