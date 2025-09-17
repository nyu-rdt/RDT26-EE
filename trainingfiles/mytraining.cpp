#include <string>
#include <vector>
#include <iostream>

using namespace std;

class NYUStudent{
    friend ostream& operator<<(ostream& os, const NYUStudent& rhs){
        const string sep = ", ";
        os << "NYUStudent: " << rhs.name << sep << rhs.N_num << sep 
           << rhs.gpa << sep << rhs.grade << endl;
        for (size_t index = 0; index < rhs.courses.size(); ++index){
            os << "Course: " << rhs.courses[index] << endl;
        }
        return os;
    }
    private:
        string name;
        string N_num;
        float gpa;
        string grade;
        vector<string> courses;

    public:
        NYUStudent(const string& name, 
                   const string& N_num, 
                   float gpa, 
                   const string& grade, 
                   const vector<string>& courses)
            : name(name), N_num(N_num), gpa(gpa), grade(grade), courses(courses) {}

        bool enroll(const string& course){
            for (size_t index = 0; index < courses.size(); ++index){
                if (courses[index] == course) {
                    cout << "Already enrolled";
                    return false;
                }
            }
            courses.push_back(course);
            return true;
        }
        bool withdraw(const string& course){
            for (size_t index = 0; index < courses.size(); ++index){
                if (courses[index] == course) {
                    courses.erase(courses.begin() + index);
                    return true;
                }
            }
            cout << "Course " << course << " not found";
            return false;
        }

        bool advance() {
            if (grade == "Freshman") {
                grade = "Sophomore";
            } else if (grade == "Sophomore") {
                grade = "Junior";
            } else if (grade == "Junior") {
                grade = "Senior";
            } else if (grade == "Senior" || grade == "Alumni") {
                grade = "Alumni";
            } else {
                cout << "Invalid grade: " << grade;
                return false;
            }
            return true;
        }

};

int main() {
    NYUStudent jason = NYUStudent("Jason", "N123", 4.0, "Sophomore", {"Course A"});
    cout << "Before: " << jason << endl;
    jason.withdraw("Course A");
    jason.enroll("ECE2004 Circuits");
    jason.enroll("CS2204 Digital Logic");
    jason.enroll("PH2121 Physics Lab");
    jason.enroll("MA2114 Calculus 3");
    jason.enroll("STS2144 Ethics and Technology");
    jason.advance();
    cout << "After: " << jason << endl;
}