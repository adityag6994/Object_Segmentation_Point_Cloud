#include <iostream>

using namespace std;

int global=0;

int main(){

    while(global < 100){
    global++;


    if(global%10 == 0 )
        cout << global << endl;

}

}