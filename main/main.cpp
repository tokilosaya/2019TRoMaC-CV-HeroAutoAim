#include "iostream"
#include "ThreadControl.h"

using namespace std;

int main(){
	processer process;
	thread t1(&processer::ArmorDetectorThreadLoop, &process);
	thread t2(&processer::InfoReadThreadLoop, &process);
	t1.join();
	t2.join();
	return 0;
}
