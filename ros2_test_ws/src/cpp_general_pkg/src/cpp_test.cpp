
#include <string>
#include <iostream>

class Person {
public:
    Person(const std::string& name, int age) 
        : m_name(name), m_age(age)
    {
        // 构造函数体
    }
    std::string m_name;
    int m_age;
};

int main() {
    Person p("Alice", 18);
    std::cout << p.m_name << std::endl;
    return 0;
}