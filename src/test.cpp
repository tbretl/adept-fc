#include <iostream>


int main() {

    int n, c, first = 0, second = 1, next;

    std::cout << "Enter the number of terms in the Fibonacci series you want" << std::endl;

    std::cin >> n;

    for ( c = 0 ; c < n ; c++ )
    {
        if ( c <= 1 )
            next = c;
        else
        {
            next = first + second;
            first = second;
            second = next;
        }
        cout << next << endl;
    }

    return 0;
}





