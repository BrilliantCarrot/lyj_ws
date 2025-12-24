#ifndef PERSON_H
#define PERSON_H

#include <string>

class Person {
public:
    Person(const std::string& firstName, const std::string& lastName);
    
    const std::string& getFirstName() const;
    void setFirstName(const std::string& firstName);
    
    const std::string& getLastName() const;
    void setLastName(const std::string& lastName);

private:
    std::string m_firstName;
    std::string m_lastName;
};

#endif // PERSON_H