#pragma once
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// A tiny JSON reader, just enough for vehicle/vehicle.json (numbers, strings, true/false/null,
// arrays and objects). No dependencies, so it also builds anywhere g++ does.
struct Json {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    double number = 0;
    bool boolean = false;
    std::string text;
    std::vector<Json> items;                // for arrays
    std::map<std::string, Json> members;    // for objects

    bool has(const std::string& key) const { return type == Object && members.count(key) > 0; }
    bool isNull() const { return type == Null; }

    // v["name"] looks up a member; a missing one is a fatal error (the vehicle file is generated, so a hole is a bug)
    const Json& operator[](const std::string& key) const {
        auto it = members.find(key);
        if (it == members.end()) {
            std::cerr << "vehicle.json: missing '" << key << "'\n";
            std::exit(1);
        }
        return it->second;
    }
    const Json& operator[](size_t i) const { return items.at(i); }
    size_t size() const { return type == Array ? items.size() : members.size(); }
    double num() const { return number; }
    int integer() const { return (int)number; }
    const std::string& str() const { return text; }
    std::vector<double> numbers() const {
        std::vector<double> out;
        for (const Json& j : items) out.push_back(j.number);
        return out;
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& s) : s(s) {}
    Json parse() {
        Json v = value();
        skip();
        return v;
    }

private:
    const std::string& s;
    size_t i = 0;

    void skip() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) i++;
    }
    [[noreturn]] void fail(const char* what) {
        std::cerr << "JSON error at offset " << i << ": " << what << "\n";
        std::exit(1);
    }
    std::string string() {
        std::string out;
        i++;  // opening quote
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\' && i + 1 < s.size()) {
                i++;
                char c = s[i];
                out += c == 'n' ? '\n' : c == 't' ? '\t' : c;  // \uXXXX never appears in our files
            } else {
                out += s[i];
            }
            i++;
        }
        i++;
        return out;
    }
    Json value() {
        skip();
        Json v;
        if (i >= s.size()) fail("unexpected end");
        char c = s[i];
        if (c == '{') {
            v.type = Json::Object;
            i++;
            skip();
            while (s[i] != '}') {
                skip();
                std::string key = string();
                skip();
                if (s[i] != ':') fail("expected :");
                i++;
                v.members[key] = value();
                skip();
                if (s[i] == ',') i++;
                skip();
            }
            i++;
        } else if (c == '[') {
            v.type = Json::Array;
            i++;
            skip();
            while (s[i] != ']') {
                v.items.push_back(value());
                skip();
                if (s[i] == ',') i++;
                skip();
            }
            i++;
        } else if (c == '"') {
            v.type = Json::String;
            v.text = string();
        } else if (s.compare(i, 4, "true") == 0) {
            v.type = Json::Bool; v.boolean = true; i += 4;
        } else if (s.compare(i, 5, "false") == 0) {
            v.type = Json::Bool; i += 5;
        } else if (s.compare(i, 4, "null") == 0) {
            i += 4;
        } else {
            v.type = Json::Number;
            char* end = nullptr;
            v.number = std::strtod(s.c_str() + i, &end);
            if (end == s.c_str() + i) fail("bad number");
            i = end - s.c_str();
        }
        return v;
    }
};

inline Json loadJson(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        std::cerr << "Could not open " << path << "\n";
        std::exit(1);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();
    return JsonParser(text).parse();
}
