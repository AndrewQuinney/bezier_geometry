#include "StaticVector.hpp"
#include "TestUtils.hpp"

class TestClass {
public:
  TestClass(const std::string &input) : myString(input) {
    std::cout << "CONSTRUCTOR: " << myString << std::endl;
  }

  TestClass(const TestClass &input) : myString(input.myString) {
    std::cout << "COPY CONSTRUCTOR: " << myString << std::endl;
  }

  TestClass(TestClass &&input) : myString(std::move(input.myString)) {
    std::cout << "MOVE CONSTRUCTOR: " << myString << std::endl;
  }

  ~TestClass() { std::cout << "DESTRUCTOR: " << myString << std::endl; }

  TestClass &operator=(const TestClass &input) {
    std::cout << "ASSIGNMENT: " << myString << "->" << input.myString
              << std::endl;
    myString = input.myString;
    return *this;
  }

  bool operator==(const TestClass &input) const {
    return myString == input.myString;
  }

  const std::string &getString() const { return myString; }

private:
  std::string myString;
};

std::ostream &operator<<(std::ostream &os, const TestClass &input) {
  os << input.getString();
  return os;
}

void basicFunctionalityTest() {
  TEST_START {
    bezier_geometry::StaticVector<std::string, 3> sv;
    CHECK(sv.size(), 0);
    CHECK(sv.begin(), sv.end());
  }
  {
    bezier_geometry::StaticVector<std::string, 3> sv;
    CHECK(sv.size(), 0);
    sv.push_back("A");
    CHECK(sv.size(), 1);
    sv.push_back("B");
    CHECK(sv.size(), 2);
    sv.push_back("C");
    CHECK(sv.size(), 3);
  }
  {
    bezier_geometry::StaticVector<std::string, 3> sv;
    sv.push_back("A");
    sv.push_back("B");
    sv.push_back("C");
    {
      std::vector<std::string> testOutput;
      for (const std::string &current : sv) {
        testOutput.push_back(current);
      }
      CHECK(testOutput.size(), 3);
      CHECK(testOutput[0], std::string("A"));
      CHECK(testOutput[1], std::string("B"));
      CHECK(testOutput[2], std::string("C"));
    }
    {
      std::vector<std::string> testOutput;
      for (const std::string &current :
           *static_cast<const bezier_geometry::StaticVector<std::string, 3> *>(
               &sv)) {
        testOutput.push_back(current);
      }
      CHECK(testOutput.size(), 3);
      CHECK(testOutput[0], std::string("A"));
      CHECK(testOutput[1], std::string("B"));
      CHECK(testOutput[2], std::string("C"));
    }
  }
  {
    bezier_geometry::StaticVector<std::string, 4> sv;
    sv.push_back("A");
    sv.push_back("B");
    sv.push_back("C");
    sv.push_back("D");
    CHECK(sv.size(), 4);
    {
      bezier_geometry::StaticVector<std::string, 4>::iterator test = sv.begin();
      std::advance(test, 1);
      sv.erase(test);
    }
    CHECK(sv.size(), 3);
    CHECK(*sv.begin(), "A");
    CHECK(*std::next(sv.begin()), "C");
    {
      bezier_geometry::StaticVector<std::string, 4>::const_iterator test =
          sv.begin();
      std::advance(test, 1);
      sv.erase(test);
    }
    CHECK(sv.size(), 2);
    CHECK(*sv.begin(), "A");
  }
  {
    bezier_geometry::StaticVector<std::string, 3> sv;
    sv.push_back("A");
    sv.push_back("B");
    sv.push_back("C");
    bezier_geometry::StaticVector<std::string, 3> copy(sv);
    CHECK(copy.size(), 3);
    CHECK(*copy.begin(), "A");
  }
  {
    bezier_geometry::StaticVector<std::string, 3> sv;
    sv.push_back("A");
    sv.push_back("B");
    sv.push_back("C");
    bezier_geometry::StaticVector<std::string, 3> copy(std::move(sv));
    CHECK(copy.size(), 3);
    CHECK(*copy.begin(), "A");
  }
  {
    bezier_geometry::StaticVector<std::string, 3> sv;
    sv.push_back("A");
    sv.push_back("B");
    sv.push_back("C");
    sv.erase(std::next(sv.begin()));
    sv.push_back("D");
    CHECK(*sv.begin(), "A");
    CHECK(*std::next(sv.begin()), "C");
    CHECK(*std::next(std::next(sv.begin())), "D");
  }
  TEST_END
}

void primitiveTest() {
  TEST_START {
    bezier_geometry::StaticVector<int, 4> sv;
    sv.push_back(6);
    sv.push_back(7);
    sv.push_back(8);
    CHECK(sv.size(), 3);
    sv.push_back(9);
    CHECK(sv.size(), 4)
    sv.erase(sv.begin());
    CHECK(sv.size(), 3)
  }
  TEST_END
}

void copyMoveTest() {
  TEST_START
  std::cout << "* START CASE 01" << std::endl;
  {
    bezier_geometry::StaticVector<TestClass, 4> sv;
    sv.push_back(TestClass("A"));
  }
  std::cout << "* END CASE 01" << std::endl;
  std::cout << "* START CASE 02" << std::endl;
  {
    bezier_geometry::StaticVector<TestClass, 4> sv;
    sv.push_back(TestClass("A"));
    std::cout << "BEFORE MOVE" << std::endl;
    bezier_geometry::StaticVector<TestClass, 4> sv2(std::move(sv));
    CHECK(sv2.size(), 1);
    CHECK(*sv2.begin(), TestClass("A"));
  }
  std::cout << "* END CASE 02" << std::endl;
  std::cout << "* START CASE 03" << std::endl;
  {
    bezier_geometry::StaticVector<TestClass, 4> sv1;
    sv1.push_back(TestClass("A"));
    bezier_geometry::StaticVector<TestClass, 4> sv2;
    CHECK(sv2.size(), 0);
    CHECK(sv1 == sv2, false);
    sv2 = sv1;
    CHECK(sv2.size(), 1);
    CHECK(sv1 == sv2, true);
    CHECK(*sv2.begin(), TestClass("A"));
  }
  std::cout << "* END CASE 03" << std::endl;
  std::cout << "* START CASE 04" << std::endl;
  {
    bezier_geometry::StaticVector<TestClass, 4> sv1;
    sv1.push_back(TestClass("A"));
    bezier_geometry::StaticVector<TestClass, 4> sv2;
    CHECK(sv2.size(), 0);
    sv2 = std::move(sv1);
    CHECK(sv2.size(), 1);
    CHECK(*sv2.begin(), TestClass("A"));
  }
  std::cout << "* END CASE 04" << std::endl;
  TEST_END
}

void initializerTest() {
  TEST_START {
    bezier_geometry::StaticVector<int, 4> sv({7});
    CHECK(sv.size(), 1)
    CHECK(*sv.begin(), 7);
  }
  {
    bezier_geometry::StaticVector<int, 4> sv = {7};
    CHECK(sv.size(), 1)
    CHECK(*sv.begin(), 7);
  }
  TEST_END
}

int main(int argc, char **argv) {
  std::cout << "%SUITE_STARTING% StaticVectorTest" << std::endl;
  std::cout << "%SUITE_STARTED%" << std::endl;
  basicFunctionalityTest();
  primitiveTest();
  copyMoveTest();
  initializerTest();
  std::cout << "%SUITE_FINISHED% time=0" << std::endl;
  return (EXIT_SUCCESS);
}
