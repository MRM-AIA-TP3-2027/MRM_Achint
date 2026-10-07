#include <iostream>
#include <vector>

int main() {
	std::vector<float> vector1(100);
	float *array1 = vector1.data();
	return 0;
}




#include <cstdlib>
#include <cmath>
#include <cassert>
#include <limits>

int main(void)
{
    //Asserts floating point compatibility at compile time
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 required");

    //C99
    float negative_infinity1 = -INFINITY;
    float negative_infinity2 = -1 * INFINITY;

    float negative_infinity3 = -std::numeric_limits<float>::infinity();
    float negative_infinity4 = -1 * std::numeric_limits<float>::infinity();

    assert(std::isinf(negative_infinity1) && negative_infinity1 < std::numeric_limits<float>::lowest());
    assert(std::isinf(negative_infinity2) && negative_infinity2 < std::numeric_limits<float>::lowest());
    assert(std::isinf(negative_infinity3) && negative_infinity3 < std::numeric_limits<float>::lowest());
    assert(std::isinf(negative_infinity4) && negative_infinity4 < std::numeric_limits<float>::lowest());

    return EXIT_SUCCESS;
}
