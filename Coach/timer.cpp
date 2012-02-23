#include <iostream>
#include <unistd.h>

using namespace std;

int main(int argv, char** argc){
	while(true){
		usleep(1500*1000);
		cout << "test" << endl;
	}
}
