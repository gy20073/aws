/*
	THIS FILE IS A PART OF GTA V SCRIPT HOOK SDK
				http://dev-c.com			
			(C) Alexander Blade 2015
*/

#include "lib/script.h"
#include "Server.h"

void ScriptMain()
{
	Server server(8000);
	freopen("outer_script.log", "w", stdout);
	int nosend = 0;
	while (true) {
		while (!server.clientConnected) {
			server.checkClient();
			scriptWait(0);
		}
		while (server.clientConnected) {
			clock_t t1 = clock();
			server.checkRecvMessage();
			clock_t t2 = clock();
			server.checkSendMessage();
			clock_t t3 = clock();
			server.scenario.run();
			clock_t t4 = clock();
			if (!server.sendOutputs) nosend++;
			printf("\ncheckRecvMessage time: %f", double(t2 - t1) / CLOCKS_PER_SEC);
			printf("\ncheckSendMessage time: %f", double(t3 - t2) / CLOCKS_PER_SEC);
			printf("\nscenario.run time: %f", double(t4 - t3) / CLOCKS_PER_SEC);

			if (double(t4 - t3) < 0.04)
				Sleep(40 - double(t4 - t3) * 1000);

			clock_t t5 = clock();

			printf("\n#### overall cycle time: %f", double(t4 - t1) / CLOCKS_PER_SEC);
			printf("\n#### overall cycle sleep time: %f", double(t5 - t1) / CLOCKS_PER_SEC);
			if (nosend > 20)
			{
				printf("\n!!!!!!!!!!!!!!!!timeout disconnect");
				server.clientConnected = false;
				nosend = 0;
			}
		}
	}
}
