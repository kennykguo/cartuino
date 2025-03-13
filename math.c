#include <stdio.h>
#include <math.h>

int main() {
    double number = 16.0;
    double base = 2.0, exponent = 3.0;
    
    // Test square root function
    double sqrt_result = sqrt(number);
    printf("The square root of %.2f is %.2f\n", number, sqrt_result);

    // Test power function
    double pow_result = pow(base, exponent);
    printf("%.2f raised to the power of %.2f is %.2f\n", base, exponent, pow_result);

    // Test sine function
    double angle = 30.0; // in degrees
    double radian = angle * (M_PI / 180.0); // convert to radians
    double sin_result = sin(radian);
    printf("The sine of %.2f degrees is %.2f\n", angle, sin_result);

    return 0;
}
