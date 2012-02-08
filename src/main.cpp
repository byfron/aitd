/* ResidualVM - A 3D game interpreter
 *
 * ResidualVM is the legal property of its developers, whose names
 * are too numerous to list here. Please refer to the AUTHORS
 * file distributed with this source distribution.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */


//////////////////////////
// FITD - Free in the Dark
//////////////////////////

// 22 septembre 2003 14:25

// seg002
#include "common/textconsole.h"

#include "fitd.h"
#include "resource.h"
#include "osystem.h"
#include "common.h"
#include "version.h"
#include "main_loop.h"
#include "common/forbidden.h"

namespace Fitd {

// TODO: Move
FitdEngine *g_fitd;

char scaledScreen[640*400];

int input5;

enumCVars *currentCVarTable = NULL;

int getCVarsIdx(enumCVars searchedType) { // TODO: optimize by reversing the table....
	return g_fitd->getCVarsIdx(searchedType);

}

void updateInHand(int objIdx) {
	int var_2;
	int actorIdx;
	int lifeOffset;
	int currentActorIdx;
	int currentActorLifeIdx;
	int currentActorLifeNum;
	int foundLife;
	actorStruct *currentActorPtr;
	actorStruct *currentActorLifePtr;

	if(objIdx == -1)
		return;

	foundLife = objectTable[objIdx].foundLife;

	if(objectTable[objIdx].foundLife == -1)
		return;

	currentActorPtr = currentProcessedActorPtr;
	currentActorIdx = currentProcessedActorIdx;
	currentActorLifeIdx = currentLifeActorIdx;
	currentActorLifePtr = currentLifeActorPtr;
	currentActorLifeNum = currentLifeNum;

	if(currentLifeNum != -1) {
		lifeOffset = (currentLifePtr - listLife->get(currentActorLifeNum)) / 2;
	}

	var_2 = 0;

	actorIdx = objectTable[objIdx].ownerIdx;

	if(actorIdx == -1) {
		actorStruct *currentActorEntryPtr = &actorTable[NUM_MAX_ACTOR-1];
		int currentActorEntry = NUM_MAX_ACTOR - 1;

		while(currentActorEntry >= 0) {
			if(currentActorEntryPtr->field_0 == -1)
				break;

			currentActorEntryPtr--;
			currentActorEntry--;
		}

		if(currentActorEntry == -1) { // no space, we will have to overwrite the last actor !
			currentActorEntry = NUM_MAX_ACTOR - 1;
			currentActorEntryPtr = &actorTable[NUM_MAX_ACTOR-1];
		}

		actorIdx = currentActorEntry;
		var_2 = 1;

		currentProcessedActorPtr = &actorTable[actorIdx];
		currentLifeActorPtr = &actorTable[actorIdx];
		currentProcessedActorIdx = actorIdx;
		currentLifeActorIdx = actorIdx;

		currentProcessedActorPtr->field_0 = objIdx;
		currentProcessedActorPtr->life = -1;
		currentProcessedActorPtr->bodyNum = -1;
		currentProcessedActorPtr->flags = 0;
		currentProcessedActorPtr->trackMode = -1;
		currentProcessedActorPtr->room = -1;
		currentProcessedActorPtr->lifeMode = -1;
		currentProcessedActorPtr->ANIM = -1;
	}

	processLife(foundLife);

	if(var_2) {
		currentProcessedActorPtr->field_0 = -1;
	}

	currentProcessedActorPtr = currentActorPtr;
	currentProcessedActorIdx = currentActorIdx;
	currentLifeActorIdx = currentActorLifeIdx;
	currentLifeActorPtr = currentActorLifePtr;

	if(currentActorLifeNum != -1) {
		currentLifeNum = currentActorLifeNum;
		currentLifePtr = listLife->get(currentLifeNum) + lifeOffset * 2;
	}
}

void sysInitSub1(char *var0, char *var1) {
	screenSm1 = var0;
	screenSm2 = var0;

	screenSm3 = var1;
	screenSm4 = var1;
	screenSm5 = var1;
}

void allocTextes(void) {
	int currentIndex;
	char *currentPosInTextes;
	int textCounter;
	int stringIndex;
	char *stringPtr;
	int textLength;

	tabTextes = (textEntryStruct *)malloc(NUM_MAX_TEXT_ENTRY * sizeof(textEntryStruct)); // 2000 = 250 * 8

	ASSERT_PTR(tabTextes);

	if(!tabTextes) {
		theEnd(1, "TabTextes");
	}

	// setup languageNameString
	if(g_fitd->getGameType() == GType_AITD3) {
		strcpy(languageNameString, "TEXTES");
	} else {
		int i = 0;

		while(languageNameTable[i]) {
			char tempString[20];

			strcpy(tempString, languageNameTable[i]);
			strcat(tempString, ".PAK");

			if(g_resourceLoader->getFileExists(tempString)) {
				strcpy(languageNameString, languageNameTable[i]);
				break;
			}

			i++;
		}
	}

	if(!languageNameString[0]) {
		error("Unable to detect language file..\n");
	}

	systemTextes = g_resourceLoader->loadPakSafe(languageNameString, 0); // todo: use real language name
	textLength = getPakSize(languageNameString, 0);

	for(currentIndex = 0; currentIndex < NUM_MAX_TEXT_ENTRY; currentIndex++) {
		tabTextes[currentIndex].index = -1;
		tabTextes[currentIndex].textPtr = NULL;
		tabTextes[currentIndex].width = 0;
	}

	currentPosInTextes = systemTextes;

	textCounter = 0;

	while(currentPosInTextes < systemTextes + textLength) {
		currentIndex = *(currentPosInTextes++);

		if(currentIndex == 26)
			break;

		if(currentIndex == '@') { // start of string marker
			stringIndex = 0;

			while((currentIndex = *(currentPosInTextes++)) >= '0' && currentIndex <= '9') { // parse string number
				stringIndex = stringIndex * 10 + currentIndex - 48;
			}

			if(currentIndex == ':') { // start of string
				stringPtr = currentPosInTextes;

				do {
					currentPosInTextes ++;
				} while((unsigned char)*(currentPosInTextes - 1) >= ' '); // detect the end of the string

				*(currentPosInTextes - 1) = 0; // add the end of string

				tabTextes[textCounter].index = stringIndex;
				tabTextes[textCounter].textPtr = stringPtr;
				tabTextes[textCounter].width = computeStringWidth(stringPtr);

				textCounter++;
			}

			if(currentIndex == 26) {
				return;
			}
		}
	}
}

void freeAll(void) {
	/*  HQR_Free(hqrUnk);

	 HQR_Free(listSamp);

	 HQR_Free(listMus);

	 free(languageData);

	 free(tabTextes);

	 free(priority);

	 free(aitdBoxGfx);

	 free(fontData);

	 free(bufferAnim);

	 if(aux != aux3)
	 {
	 free(aux);
	 }

	 free(aux2);*/

	//TODO: implement all the code that restore the interrupts & all
}

textEntryStruct *getTextFromIdx(int index) {
	int currentIndex;

	for(currentIndex = 0; currentIndex < NUM_MAX_TEXT_ENTRY; currentIndex++) {
		if(tabTextes[currentIndex].index == index) {
			return(&tabTextes[currentIndex]);
		}
	}

	return(NULL);
}

void fillBox(int x1, int y1, int x2, int y2, char color) { // fast recode. No RE
	int width = x2 - x1 + 1;
	int height = y2 - y1 + 1;

	char *dest = screen + y1 * 320 + x1;

	for(int i = 0; i < height; i++) {
		for(int j = 0; j < width; j++) {
			*(dest++) = color;
		}

		dest += 320 - width;
	}
}

void selectHeroSub1(int x1, int y1, int x2, int y2) {
	for(int i = y1; i < y2; i++) {
		for(int j = x1; j < x2; j++) {
			*(screenSm3 + i * 320 + j) = *(screenSm1 + i * 320 + j);
		}
	}
}

int selectHero(void) {
	int choice = 0;
	int var_4 = 1;
	int choiceMade = 0;

	sysInitSub1(aux, screen);

	while(choiceMade == 0) {
		process_events();
		readKeyboard();

		loadPakToPtr("ITD_RESS", 10, aux);
		copyToScreen(aux, screen);
		copyToScreen(screen, aux2);

		if(choice == 0) {
			drawAITDBox(80, 100, 160, 200);
			selectHeroSub1(10, 10, 149, 190);
		} else {
			drawAITDBox(240, 100, 160, 200);
			selectHeroSub1(170, 10, 309, 190);
		}

		flipScreen();

		if(var_4 != 0) {
			make3dTatouUnk1(0x40, 0);

			do {
				readKeyboard();
			} while(input1 || input2);

			var_4 = 0;
		}

		while((input3 = input2) != 28 && input1 == 0) { // process input
			readKeyboard();
			process_events();

			if(inputKey & 4) { // left
				choice = 0;
				copyToScreen(aux2, screen);
				drawAITDBox(80, 100, 160, 200);
				selectHeroSub1(10, 10, 149, 190);
				flipScreen();

				while(inputKey != 0) {
					readKeyboard();
				}
			}

			if(inputKey & 8) { // right
				choice = 1;
				copyToScreen(aux2, screen);
				drawAITDBox(240, 100, 160, 200);
				selectHeroSub1(170, 10, 309, 190);
				flipScreen();

				while(inputKey != 0) {
					readKeyboard();
				}
			}

			if(input3 == 1) {
				sysInitSub1(aux2, screen);
				fadeOut(0x40, 0);
				return(-1);
			}
		}

		fadeOut(0x40, 0);
		readVar = 0;

		switch(choice) {
		case 0: {
			copyToScreen(unkScreenVar, screen);
			setClipSize(0, 0, 319, 199);
			loadPakToPtr("ITD_RESS", 14, aux);
			selectHeroSub1(160, 0, 319, 199);
			copyToScreen(screen, aux);
			printText(CVars[getCVarsIdx(INTRO_HERITIERE)] + 1, 165, 5, 314, 194, 2, 15);
			CVars[getCVarsIdx(CHOOSE_PERSO)] = 1;
			break;
		}
		case 1: {
			copyToScreen(unkScreenVar, screen);
			setClipSize(0, 0, 319, 199);
			loadPakToPtr("ITD_RESS", 14, aux);
			selectHeroSub1(0, 0, 159, 199);
			copyToScreen(screen, aux);
			printText(CVars[getCVarsIdx(INTRO_DETECTIVE)] + 1, 5, 5, 154, 194, 2, 15);
			CVars[getCVarsIdx(CHOOSE_PERSO)] = 0;
			break;
		}
		}

		if(input3 && 0x1C) {
			choiceMade = 1;
		}

	}

	fadeOut(0x40, 0);

	sysInitSub1(aux2, screen);

	return(choice);
}

void printTextSub5(int x, int y, int param, char *gfx) {
}

void printTextSub6(hqrEntryStruct *hqrPtr, int index) {
}

void printTextSub7() {
}

void printTextSub8() {
}

int printText(int index, int left, int top, int right, int bottom, int mode, int color) {
	bool lastPageReached = false;
	char tabString[] = "    ";
	int var_12 = 1;
	int currentPage = 0;
	int quit = 0;
	int var_2 = -1;
	int var_1C3;
	char *localTextTable[100];
	int currentTextIdx;
	int maxStringWidth;
	int var_8;
	char *textPtr;

	initFont(fontData, color);

	maxStringWidth = right - left + 4;

	var_8 = hqrUnk->printTextSub1(getPakSize(languageNameString, index) + 300);

	textPtr = hqrUnk->printTextSub2(var_8);

	if(!loadPakToPtr(languageNameString, index, textPtr)) {
		theEnd(1, languageNameString);
	}

	localTextTable[0] = textPtr;

	//  soundVar2 = -1;
	//  soundVar1 = -1;

	while(!quit) {
		char *var_1C2;
		int currentTextY;
		copyToScreen(aux, screen);
		process_events();
		setClipSize(left, top, right, bottom);

		var_1C2 = localTextTable[currentPage];

		currentTextY = top;
		lastPageReached = false;

		while(currentTextY <= bottom - 16) {
			int var_1AA = 1;
			int var_1BA = 0;
			int currentStringWidth;
			int currentTextX;

			regularTextEntryStruct *currentText = textTable;

			int numWordInLine = 0;

			int interWordSpace = 0;

parseSpe:
			while(*var_1C2 == '#') {
				//char* var_1BE = var_1C2;
				var_1C2++;

				switch(*(var_1C2++)) {
				case 'P': { // page change
					if(currentTextY > top) // Hu ?
						goto pageChange;
					break;
				}
				case 'T': { // tab
					currentText->textPtr = tabString;
					currentText->width = computeStringWidth(currentText->textPtr) + 3;
					var_1BA += currentText->width;
					numWordInLine++;
					currentText++;
					break;
				}
				case 'C': { // center
					var_1AA &= 0xFFFE;
					var_1AA |= 8;
					break;
				}
				case 'G': { // print number
					currentTextIdx = 0;

					while(*var_1C2 >= '0' && *var_1C2 <= '9') {
						currentTextIdx = (currentTextIdx * 10 + *var_1C2 - 48);
						var_1C2 ++;
					}

					if(loadPakToPtr("ITD_RESS", 9, aux2)) {
						/*  var_C = printTextSub3(currentTextIdx,aux2);
						 var_A = printTextSub4(currentTextIdx,aux2);

						 if(currentTextY + var_A > bottom)
						 {
						 var_1C2 = var_1BE;

						 goto pageChange;
						 }
						 else
						 {
						 printTextSub5((((right-left)/2)+left)-var_C, currentTextY, currentTextIdx, aux2);
						 currentTextY = var_A;
						 }*/
					}

					break;
				}
				}
			}

			currentText->textPtr = var_1C2;

			do {
				var_1C3 = *((unsigned char *)var_1C2++);
			} while(var_1C3 > ' '); // go to the end of the string

			*(var_1C2 - 1) = 0; // add end of string marker to cut the word

			currentStringWidth = computeStringWidth(currentText->textPtr) + 3;

			if(currentStringWidth <= maxStringWidth) {
				if(var_1BA + currentStringWidth > maxStringWidth) {
					var_1C2 = currentText->textPtr;
				} else {
					currentText->width = currentStringWidth;
					var_1BA += currentStringWidth;

					numWordInLine++;
					currentText++;

					switch(var_1C3) { // eval the character that caused the 'end of word' state
					case 0x1A: { // end of text
						var_1AA &= 0xFFFE;
						var_1AA |= 4;
						lastPageReached = true;

						break;
					}
					case 0x0D:
					case 0x00: {
						if(*var_1C2 < ' ') {
							if(*(++var_1C2) == 0xD) {
								var_1C2 += 2;
								var_1AA &= 0xFFFE;
								var_1AA |= 2;
							} else {
								if(*var_1C2 == '#') {
									var_1AA &= 0xFFFE;
								}
							}
						} else {
							goto parseSpe;
						}
						break;
					}
					default: {
						goto parseSpe;
						break;
					}
					}
				}
			} else {
				quit = 1;
			}

			if(var_1AA & 1) { // stretch words on line
				//interWordSpace = (maxStringWidth - var_1BA) / (numWordInLine-1);
			}

			currentText = textTable;
			currentTextX;

			if(var_1AA & 8) { // center
				currentTextX = left + ((maxStringWidth - var_1BA) / 2);
			} else {
				currentTextX = left;
			}

			currentTextIdx = 0;

			while(currentTextIdx < numWordInLine) {
				renderText(currentTextX, currentTextY, screen, currentText->textPtr);

				currentTextX += currentText->width + interWordSpace; // add inter word space

				currentText++;
				currentTextIdx++;
			}

			if(var_1AA & 2) { // font size
				currentTextY += 8;
			}

			currentTextY += 16;
		}

		if(!lastPageReached || bottom + 16 < currentTextY) {
pageChange:
			if(lastPageReached) {
				*(var_1C2 - 1) = 0x1A;
			} else {
				localTextTable[currentPage+1] = var_1C2;
			}

			if(mode == 0) {
				if(currentPage > 0) {
					printTextSub5(left - 19, 185, 12, aitdBoxGfx);
				}

				if(!lastPageReached) {
					printTextSub5(right + 4, 185, 11, aitdBoxGfx);
				}
			}

			if(mode == 2) {
				if(currentPage > 0) {
					printTextSub5(left - 3, 191, 13, aitdBoxGfx);
				}

				if(!lastPageReached) {
					printTextSub5(right - 10, 191, 14, aitdBoxGfx);
				}
			}

			if(var_12) {
				if(mode != 1) {
					flipScreen();
					make3dTatouUnk1(16, 0);
				} else {
					if(readVar) {
						printTextSub7();
					} else {
						flipScreen();
					}
				}

				var_12 = 0;
			} else {
				if(readVar) {
					if(var_2 < currentPage) {
						printTextSub7();
					} else {
						printTextSub8();
					}
				} else {
					flipScreen();
				}
			}

			if(mode != 1) { // mode != 1: normal behavior (user can flip pages)
				do {
					readKeyboard();
				} while(input2 || inputKey || input1);

				input3 = input2 & 0xFF;
				input4 = inputKey & 0xFF;
				button = input1;

				if(input3 == 1 || button) {
					quit = 1;
				} else {
					if(mode != 2 || input3 != 0x1C) {
						if(inputKey & 0xA0 || input3 == 0x1C) {
							if(!lastPageReached) { // flip to next page
								var_2 = currentPage++;

								if(mode == 2) {
									playSound(CVars[getCVarsIdx(SAMPLE_PAGE)]);
									//                        soundVar2 = -1;
									//                        soundVar1 = -1;
								}
							}
						} else {
							// flip to previous page

							// TODO: implement...
						}
					} else {
						quit = 1;
					}
				}
			} else { // auto page fip
				unsigned int var_6;
				startChrono(&var_6);

				do {
					process_events();
					readKeyboard();
					if(evalChrono(&var_6) > 300) {
						break;
					}
				} while(!input2 && !input1);

				if(input2 || input1) {
					quit = 1;
				}

				if(!lastPageReached) {
					currentPage++;
					playSound(CVars[getCVarsIdx(SAMPLE_PAGE)]);
					//                soundVar2 = -1;
				} else {
					quit = 1;
					mode = 0;
				}
			}
		}
	}

	printTextSub6(hqrUnk, var_8);

	return(mode);
}

void makeIntroScreens(void) {
	char *data;
	unsigned int chrono;

	data = loadPak("ITD_RESS", 13);
	copyToScreen(data + 770, unkScreenVar);
	make3dTatouUnk1(8, 0);
	memcpy(screen, unkScreenVar, 320 * 200);
	flipScreen();
	free(data);
	loadPakToPtr("ITD_RESS", 7, aux);
	startChrono(&chrono);

	do {
		int time;

		process_events();
		readKeyboard();

		time = evalChrono(&chrono);

		if(time >= 0x30)
			break;

	} while(input2 == 0 && input1 == 0);

	playSound(CVars[getCVarsIdx(SAMPLE_PAGE)]);
	/*  soundVar2 = -1;
	 soundVar1 = -1;
	 soundVar2 = -1;
	 soundVar1 = 0; */
	//  readVar = 1;
	printText(CVars[getCVarsIdx(TEXTE_CREDITS)] + 1, 48, 2, 260, 197, 1, 26);
}

void initEngine() {
	int32 choosePersoBackup;

	Common::SeekableReadStream *stream = g_resourceLoader->getFile("OBJETS.ITD");
	
	if(!stream)
		theEnd(0, "OBJETS.ITD");

	maxObjects = stream->readUint16LE();

	if(g_fitd->getGameType() == GType_AITD1) {
		objectTable = new objectStruct[300];
	} else {
		objectTable = new objectStruct[maxObjects];
	}

	for(int i = 0; i < maxObjects; i++) {
		objectTable[i].readFromStream(stream);
	}
	delete stream;

	/* fHandle = fopen("objDump.txt","w+");
	 for(i=0;i<maxObjects;i++)
	 {
	 fprintf(fHandle,"Object %d:", i);

	 fprintf(fHandle,"\t body:%03d",objectTable[i].field_2);
	 fprintf(fHandle,"\t anim:%03d",objectTable[i].anim);
	 fprintf(fHandle,"\t stage:%01d",objectTable[i].stage);
	 fprintf(fHandle,"\t room:%02d",objectTable[i].room);
	 fprintf(fHandle,"\t lifeMode: %01d",objectTable[i].lifeMode);
	 fprintf(fHandle,"\t life: %02d",objectTable[i].life);
	 fprintf(fHandle,"\t beta: %03X",objectTable[i].beta);

	 fprintf(fHandle,"\n");
	 }
	 fclose(fHandle);

	 fHandle = fopen("objNames.txt","w+");
	 for(i=0;i<maxObjects;i++)
	 {
	 fprintf(fHandle,"obj%03d ",i);
	 if(objectTable[i].foundName == -1)
	 {
	 fprintf(fHandle,"-1\n");
	 }
	 else
	 {
	 textEntryStruct* name = getTextFromIdx(objectTable[i].foundName);
	 if(name)
	 fprintf(fHandle,"%s\n",name->textPtr);
	 else
	 fprintf(fHandle,"?\n");
	 }
	 }
	 fclose(fHandle); */
	//

	vars = (int16 *)g_resourceLoader->loadFromItd("VARS.ITD");

	varSize = g_resourceLoader->getFileSize("VARS.ITD");

	if(g_fitd->getGameType() == GType_AITD1) {
		choosePersoBackup = CVars[getCVarsIdx(CHOOSE_PERSO)]; // backup hero selection
	}

	stream = g_resourceLoader->getFile("DEFINES.ITD");
	if(!stream) {
		theEnd(0, "DEFINES.ITD");
	}

	///////////////////////////////////////////////
	{
		for (int i = 0; i < g_fitd->getNumCVars(); i++) {
			CVars[i] = stream->readUint16LE();
		}
		delete stream;

		for(int i = 0; i < g_fitd->getNumCVars(); i++) {
			CVars[i] = ((CVars[i] & 0xFF) << 8) | ((CVars[i] & 0xFF00) >> 8);
		}
	}
	//////////////////////////////////////////////

	if(g_fitd->getGameType() == GType_AITD1) {
		CVars[getCVarsIdx(CHOOSE_PERSO)] = choosePersoBackup;
	}

	listLife = new hqrEntryStruct("LISTLIFE", 10000, 100);
	listTrack = new hqrEntryStruct("LISTTRAK", 1000, 10);

	// TODO: missing dos memory check here

	if(g_fitd->getGameType() == GType_AITD1) {
		listBody = new hqrEntryStruct(listBodySelect[CVars[getCVarsIdx(CHOOSE_PERSO)]], 100000, 50); // was calculated from free mem size
		listAnim = new hqrEntryStruct(listAnimSelect[CVars[getCVarsIdx(CHOOSE_PERSO)]], 100000, 50); // was calculated from free mem size
	} else {
		listBody = new hqrEntryStruct("LISTBODY", 100000, 50); // was calculated from free mem size
		listAnim = new hqrEntryStruct("LISTANIM", 100000, 50); // was calculated from free mem size

		listMatrix = new hqrEntryStruct("LISTMAT", 16000, 5);
	}


	for(int i = 0; i < NUM_MAX_ACTOR; i++) {
		actorTable[i].field_0 = -1;
	}

	if(g_fitd->getGameType() == GType_AITD1) {
		currentCameraTarget = CVars[getCVarsIdx(WORLD_NUM_PERSO)];
	}
}

void initVarsSub1(void) {
	for(int i = 0; i < 5; i++) {
		messageTable[i].string = NULL;
	}
}

void initVars() {
	giveUp = 0;
	if(g_fitd->getGameType() == GType_AITD1) {
		inHand = -1;
		numObjInInventory = 0;
	} else {
		int i;

		for(i = 0; i < 2; i++) {
			numObjInInventoryTable[i] = 0;
			inHandTable[i] = -1;
		}
	}

	action = 0;

	genVar1 = genVar2;
	genVar3 = genVar4;

	genVar5 = 0;
	genVar6 = 0;

	soundVar2 = -1;
	genVar7 = -1;
	soundVar1 = -1;
	currentMusic = -1;
	nextMusic = -1;

	lightVar1 = 0;
	lightVar2 = 0;

	genVar9 = -1;
	currentCameraTarget = -1;

	statusScreenAllowed = 1;

	initVarsSub1();
}

void loadCamera(int cameraIdx) {
	char name[16];
	int useSpecial = -1;

	sprintf(name, "CAMERA%02d", currentEtage);
	//strcat(name,".PAK");

	if(g_fitd->getGameType() == GType_AITD1) {
		if(CVars[getCVarsIdx(LIGHT_OBJECT)] == 1) {
			switch(currentEtage) {
			case 6: {
				if(cameraIdx == 0) {
					useSpecial = 17;
				}
				if(cameraIdx == 5) {
					useSpecial = 18;
				}
				break;
			}
			case 7: {
				if(cameraIdx == 1) {
					useSpecial = 16;
				}
				break;
			}
			}
		}

		if(useSpecial != -1) {
			strcpy(name, "ITD_RESS");
			cameraIdx = useSpecial;
		}
	}

	if(!loadPakToPtr(name, cameraIdx, aux)) {
		theEnd(0, name);
	}

	if(g_fitd->getGameType() == GType_AITD3) {
		memmove(aux, aux + 4, 64000 + 0x300);
	}

	if(g_fitd->getGameType() >= GType_JACK) {
		copyPalette(aux + 64000, g_driver->_palette);

		if(g_fitd->getGameType() == GType_AITD3) {
			//memcpy(palette,defaultPaletteAITD3,0x30);
		} else {
			g_driver->_paletteObj->loadDefault();
			g_driver->_paletteObj->convertIfRequired();
		}

		g_driver->setPalette((byte*)g_driver->_palette);
	}
}

void setupPointTransformSM(int x, int y, int z) {
	transformX = x & 0x3FF;
	if(transformX) {
		transformXCos = cosTable[transformX];
		transformXSin = cosTable[(transformX+0x100)&0x3FF];
		transformUseX = true;
	} else {
		transformUseX = false;
	}

	transformY = y & 0x3FF;
	if(transformY) {
		transformYCos = cosTable[transformY];
		transformYSin = cosTable[(transformY+0x100)&0x3FF];
		transformUseY = true;
	} else {
		transformUseY = false;
	}

	transformZ = z & 0x3FF;
	if(transformZ) {
		transformZCos = cosTable[transformZ];
		transformZSin = cosTable[(transformZ+0x100)&0x3FF];
		transformUseZ = true;
	} else {
		transformUseZ = false;
	}
}

void setupSelfModifyingCode(int x, int y, int z) {
	translateX = x;
	translateY = y;
	translateZ = z;
}

void setupSMCode(int centerX, int centerY, int x, int y, int z) {
	cameraCenterX = centerX;
	cameraCenterY = centerY;

	cameraX = x;
	cameraY = y;
	cameraZ = z;
}

int setupCameraSub1Sub1(int value) {
	char *ptr = currentCameraVisibilityList;
	int var;

	while((var = *(ptr++)) != -1) {
		if(value == var) {
			return(1);
		}
	}

	return(0);
}

// setup visibility list
void setupCameraSub1() {
	uint32 i;
	int j;
	int var_10;

	char *dataTabPos = currentCameraVisibilityList;

	*dataTabPos = -1;

	// visibility list: add linked rooms
	for(i = 0; i < roomDataTable[currentDisplayedRoom].numSceZone; i++) {
		if(roomDataTable[currentDisplayedRoom].sceZoneTable[i].type == 0) {
			var_10 = roomDataTable[currentDisplayedRoom].sceZoneTable[i].parameter;
			if(!setupCameraSub1Sub1(var_10)) {
				*(dataTabPos++) = var_10;
				*(dataTabPos) = -1;
			}
		}
	}

	// visibility list: add room seen by the current camera
	for(j = 0; j < cameraDataTable[currentCamera]->_numCameraZoneDef; j++) {
		if(!setupCameraSub1Sub1(cameraDataTable[currentCamera]->_cameraZoneDefTable[j].dummy1)) {
			*(dataTabPos++) = (char)cameraDataTable[currentCamera]->_cameraZoneDefTable[j].dummy1;
			*(dataTabPos) = -1;
		}
	}
}

void updateAllActorAndObjectsSub1(int index) { // remove actor
	actorStruct *actorPtr = &actorTable[index];

	if(actorPtr->field_0 == -2) { // flow
		actorPtr->field_0 = -1;

		if(actorPtr->ANIM == 4) {
			CVars[getCVarsIdx(FOG_FLAG)] = 0;
		}

		printTextSub6(hqrUnk, actorPtr->FRAME);
	} else {
		if(actorPtr->field_0 >= 0) {
			objectStruct *objectPtr = &objectTable[actorPtr->field_0];

			objectPtr->ownerIdx = -1;
			actorPtr->field_0 = -1;

			objectPtr->body = actorPtr->bodyNum;
			objectPtr->anim = actorPtr->ANIM;
			objectPtr->frame = actorPtr->FRAME;
			objectPtr->animType = actorPtr->field_40;
			objectPtr->animInfo = actorPtr->field_42;
			objectPtr->flags = actorPtr->flags & 0xFFF7;
			objectPtr->flags |= actorPtr->dynFlags << 5; // ???!!!?
			objectPtr->life = actorPtr->life;
			objectPtr->lifeMode = actorPtr->lifeMode;
			objectPtr->trackMode = actorPtr->trackMode;

			if(objectPtr->trackMode) {
				objectPtr->trackNumber = actorPtr->trackNumber;
				objectPtr->positionInTrack = actorPtr->positionInTrack;
				if(g_fitd->getGameType() != GType_AITD1) {
					objectPtr->mark = actorPtr->MARK;
				}
			}

			objectPtr->x = actorPtr->roomX + actorPtr->modX;
			objectPtr->y = actorPtr->roomY + actorPtr->modY;
			objectPtr->z = actorPtr->roomZ + actorPtr->modZ;

			objectPtr->alpha = actorPtr->alpha;
			objectPtr->beta = actorPtr->beta;
			objectPtr->gamma = actorPtr->gamma;

			objectPtr->stage = actorPtr->stage;
			objectPtr->room = actorPtr->room;

			actorTurnedToObj = 1;
		}
	}
}

bool pointRotateEnable = true;

int pointRotateCosX;
int pointRotateSinX;
int pointRotateCosY;
int pointRotateSinY;
int pointRotateCosZ;
int pointRotateSinZ;

void setupPointRotate(int alpha, int beta, int gamma) {
	pointRotateEnable = true;

	pointRotateCosX = cosTable[alpha&0x3FF];
	pointRotateSinX = cosTable[((alpha&0x3FF) + 0x100) & 0x3FF];

	pointRotateCosY = cosTable[beta&0x3FF];
	pointRotateSinY = cosTable[((beta&0x3FF) + 0x100) & 0x3FF];

	pointRotateCosZ = cosTable[gamma&0x3FF];
	pointRotateSinZ = cosTable[((gamma&0x3FF) + 0x100) & 0x3FF];
}

void pointRotate(int x, int y, int z, int *destX, int *destY, int *destZ) {
	if(pointRotateEnable) {
		{
			int tempX = x;
			int tempY = y;
			x = ((((tempX * pointRotateSinZ) - (tempY * pointRotateCosZ))) >> 16) << 1;
			y = ((((tempX * pointRotateCosZ) + (tempY * pointRotateSinZ))) >> 16) << 1;
		}

		{
			int tempX = x;
			int tempZ = z;

			x = ((((tempX * pointRotateSinY) - (tempZ * pointRotateCosY))) >> 16) << 1;
			z = ((((tempX * pointRotateCosY) + (tempZ * pointRotateSinY))) >> 16) << 1;
		}

		{
			int tempY = y;
			int tempZ = z;
			y = ((((tempY * pointRotateSinX) - (tempZ * pointRotateCosX))) >> 16) << 1;
			z = ((((tempY * pointRotateCosX) + (tempZ * pointRotateSinX))) >> 16) << 1;
		}

		*destX = x;
		*destY = y;
		*destZ = z;
	}
}

void zvRotSub(int X, int Y, int Z, int alpha, int beta, int gamma) {
	if(alpha || beta || gamma) {
		setupPointRotate(alpha, beta, gamma);
		pointRotate(X, Y, Z, &animMoveX, &animMoveY, &animMoveZ);
	} else {
		animMoveX = X;
		animMoveY = Y;
		animMoveZ = Z;
	}
}

void getZvRot(char *bodyPtr, ZVStruct *zvPtr, int alpha, int beta, int gamma) {
	int X1 = 32000;
	int Y1 = 32000;
	int Z1 = 32000;

	int X2 = -32000;
	int Y2 = -32000;
	int Z2 = -32000;

	int i;
	int tempX;
	int tempY;
	int tempZ;

	getZvNormal(bodyPtr, zvPtr);

	for(i = 0; i < 8; i++) {
		switch(i) {
		case 0: {
			tempX = zvPtr->ZVX1;
			tempY = zvPtr->ZVY1;
			tempZ = zvPtr->ZVZ1;
			break;
		}
		case 1: {
			tempZ = zvPtr->ZVZ2;
			break;
		}
		case 2: {
			tempX = zvPtr->ZVX2;
			break;
		}
		case 3: {
			tempZ = zvPtr->ZVZ1;
			break;
		}
		case 4: {
			tempY = zvPtr->ZVY2;
			break;
		}
		case 5: {
			tempX = zvPtr->ZVX1;
			break;
		}
		case 6: {
			tempZ = zvPtr->ZVZ2;
			break;
		}
		case 7: {
			tempX = zvPtr->ZVX2;
			break;
		}
		}

		zvRotSub(tempX, tempY, tempZ, alpha, beta, gamma);

		if(animMoveX < X1)
			X1 = animMoveX;

		if(animMoveX > X2)
			X2 = animMoveX;

		if(animMoveY < Y1)
			Y1 = animMoveY;

		if(animMoveY > Y2)
			Y2 = animMoveY;

		if(animMoveZ < Z1)
			Z1 = animMoveZ;

		if(animMoveZ > Z2)
			Z2 = animMoveZ;

	}

	zvPtr->ZVX1 = X1;
	zvPtr->ZVX2 = X2;
	zvPtr->ZVY1 = Y1;
	zvPtr->ZVY2 = Y2;
	zvPtr->ZVZ1 = Z1;
	zvPtr->ZVZ2 = Z2;
}

void copyZv(ZVStruct *source, ZVStruct *dest) {
	memcpy(dest, source, sizeof(ZVStruct));
}

void setupCameraSub4(void) {
	copyToScreen(aux, aux2);

	//TODO: implementer la suite
}

void setMoveMode(int trackMode, int trackNumber) {
	currentProcessedActorPtr->trackMode = trackMode;

	switch(trackMode) {
	case 2: {
		currentProcessedActorPtr->trackNumber = trackNumber;
		currentProcessedActorPtr->MARK = -1;
		break;
	}
	case 3: {
		currentProcessedActorPtr->trackNumber = trackNumber;
		currentProcessedActorPtr->positionInTrack = 0;
		currentProcessedActorPtr->MARK = -1;
		break;
	}
	}
}

int16 cameraVisibilityVar = 0;

int checkRoomAitd2Only(int roomNumber) {
	int i;
	int found = 0;
	int numZone = cameraDataTable[currentCamera]->_numCameraZoneDef;

	for(i = 0; i < numZone; i++) {
		if(cameraDataTable[currentCamera]->_cameraZoneDefTable[i].dummy1 == roomNumber) {
			cameraVisibilityVar = i;
			return(1);
		}
	}

	cameraVisibilityVar = -1;

	return 0;
}

int checkZoneAitd2Only(int X, int Z) { // TODO: not 100% exact
	// if(changeCameraSub1(X,X,Z,Z,&cameraDataTable[currentCamera]->cameraZoneDefTable[cameraVisibilityVar]))
	return 1;

	return 0;
}

int updateActorAitd2Only(int actorIdx) {
	actorStruct *currentActor = &actorTable[actorIdx];

	if(g_fitd->getGameType() == GType_AITD1) {
		return 0;
	}

	if(currentActor->bodyNum != -1) {
		if(checkRoomAitd2Only(currentActor->room)) {
			if(checkZoneAitd2Only(currentActor->roomX + currentActor->modX, currentActor->roomZ + currentActor->modZ)) {
				currentActor->lifeMode |= 4;
				return 1;
			}
		}
	}

	return 0;
}

void updateAllActorAndObjectsAITD2() {
	int i;
	actorStruct *currentActor = actorTable;
	objectStruct *currentObject;

	for(i = 0; i < NUM_MAX_ACTOR; i++) {
		if(currentActor->field_0 != -1) {
			currentActor->lifeMode &= 0xFFFB;

			if(currentActor->stage == currentEtage) {
				if(currentActor->life != -1) {
					switch(currentActor->lifeMode) {
					case 1: {
						break;
					}
					case 2: {
						if(currentActor->room != currentDisplayedRoom) {
							if(!updateActorAitd2Only(i)) {
								updateAllActorAndObjectsSub1(i);
							}
						}
						break;
					}
					case 3: {
						if(!setupCameraSub1Sub1(currentActor->room)) {
							if(!updateActorAitd2Only(i)) {
								updateAllActorAndObjectsSub1(i);
							}
						}
						break;
					}
					default: {
						if(!updateActorAitd2Only(i)) {
							updateAllActorAndObjectsSub1(i);
						}
						break;
					}
					}
				} else {
					if(!setupCameraSub1Sub1(currentActor->room)) {
						updateAllActorAndObjectsSub1(i);
					}
				}
			} else {
				updateAllActorAndObjectsSub1(i);
			}
		}

		currentActor++;
	}

	for(i = 0; i < maxObjects; i++) {
		currentObject = &objectTable[i];

		if(currentObject->ownerIdx != -1) {
			if(currentCameraTarget == i) {
				genVar9 = currentObject->ownerIdx;
			}
		} else {
			if(currentObject->stage == currentEtage) {
				if(currentObject->life != -1) {
					if(currentObject->lifeMode != -1) {
						int actorIdx;
						int di;

						switch(currentObject->lifeMode & 3) {
						case 0: {
							di = 0;
							break;
						}
						case 1: {
							di = 1;
							break;
						}
						case 2: {
							if(currentObject->room != currentDisplayedRoom) {
								di = 0;
							} else {
								di = 1;
							}
							break;
						}
						case 3: {
							if(!setupCameraSub1Sub1(currentObject->room)) {
								di = 0;
							} else {
								di = 1;
							}
							break;
						}
						}

						if(!di) {
							if(currentObject->body != -1) {
								if(checkRoomAitd2Only(currentObject->room)) {
									if(checkZoneAitd2Only(currentObject->x, currentObject->z)) {
										currentObject->lifeMode |= 4;
									} else {
										continue;
									}
								} else {
									continue;
								}
							} else {
								continue;
							}
						}

						//int var_C = currentObject->flags & 0xFFDF;
						//int var_E = currentObject->field_2;
						//int var_A = currentObject->anim;

addObject:
						actorIdx = copyObjectToActor(currentObject->body, currentObject->field_6, currentObject->foundName,
						                             currentObject->flags & 0xFFDF,
						                             currentObject->x, currentObject->y, currentObject->z,
						                             currentObject->stage, currentObject->room,
						                             currentObject->alpha, currentObject->beta, currentObject->gamma,
						                             currentObject->anim,
						                             currentObject->frame, currentObject->animType, currentObject->animInfo);

						currentObject->ownerIdx = actorIdx;

						if(actorIdx != -1) {
							currentProcessedActorPtr = &actorTable[actorIdx];
							currentProcessedActorIdx = actorIdx;

							if(currentCameraTarget == i) {
								genVar9 = currentProcessedActorIdx;
							}

							currentProcessedActorPtr->dynFlags = (currentObject->flags & 0x20) / 0x20; // recheck
							currentProcessedActorPtr->life = currentObject->life;
							currentProcessedActorPtr->lifeMode = currentObject->lifeMode;

							currentProcessedActorPtr->field_0 = i;

							setMoveMode(currentObject->trackMode, currentObject->trackNumber);

							currentProcessedActorPtr->positionInTrack = currentObject->positionInTrack;

							if(g_fitd->getGameType() != GType_AITD1) {
								currentProcessedActorPtr->MARK = currentObject->mark;
							}

							actorTurnedToObj = 1;
						}
					}
				} else {
					if(setupCameraSub1Sub1(currentObject->room))
						goto addObject;
				}
			}
		}
	}

	//  objModifFlag1 = 0;

	//TODO: object update
}


void updateAllActorAndObjects() {
	int i;
	actorStruct *currentActor = actorTable;
	objectStruct *currentObject;

	if(g_fitd->getGameType() >= GType_JACK) {
		updateAllActorAndObjectsAITD2();
		return;
	}

	for(i = 0; i < NUM_MAX_ACTOR; i++) {
		if(currentActor->field_0 != -1) {
			if(currentActor->stage == currentEtage) {
				if(currentActor->life != -1) {
					switch(currentActor->lifeMode) {
					case 0: {
						break;
					}
					case 1: {
						if(currentActor->room != currentDisplayedRoom) {
							updateAllActorAndObjectsSub1(i);
						}
						break;
					}
					case 2: {
						if(!setupCameraSub1Sub1(currentActor->room)) {
							updateAllActorAndObjectsSub1(i);
						}
						break;
					}
					default: {
						updateAllActorAndObjectsSub1(i);
						break;
					}
					}
				} else {
					if(!setupCameraSub1Sub1(currentActor->room)) {
						updateAllActorAndObjectsSub1(i);
					}
				}
			} else {
				updateAllActorAndObjectsSub1(i);
			}
		}

		currentActor++;
	}

	currentObject = objectTable;

	for(i = 0; i < maxObjects; i++) {
		if(currentObject->ownerIdx != -1) {
			if(currentCameraTarget == i) {
				genVar9 = currentObject->ownerIdx;
			}
		} else {
			if(currentObject->stage == currentEtage) {
				if(currentObject->life != -1) {
					if(currentObject->lifeMode != -1) {
						int actorIdx;

						switch(currentObject->lifeMode) {
						case 1: {
							if(currentObject->room != currentDisplayedRoom) {
								currentObject++;
								continue;
							}
							break;
						}
						case 2: {
							if(!setupCameraSub1Sub1(currentObject->room)) {
								currentObject++;
								continue;
							}
							break;
						}
						}

						//int var_C = currentObject->flags & 0xFFDF;
						//int var_E = currentObject->field_2;
						//int var_A = currentObject->anim;

addObject:
						actorIdx = copyObjectToActor(currentObject->body, currentObject->field_6, currentObject->foundName,
						                             currentObject->flags & 0xFFDF,
						                             currentObject->x, currentObject->y, currentObject->z,
						                             currentObject->stage, currentObject->room,
						                             currentObject->alpha, currentObject->beta, currentObject->gamma,
						                             currentObject->anim,
						                             currentObject->frame, currentObject->animType, currentObject->animInfo);

						currentObject->ownerIdx = actorIdx;

						if(actorIdx != -1) {
							currentProcessedActorPtr = &actorTable[actorIdx];
							currentProcessedActorIdx = actorIdx;

							if(currentCameraTarget == i) {
								genVar9 = currentProcessedActorIdx;
							}

							currentProcessedActorPtr->dynFlags = (currentObject->flags & 0x20) / 0x20; // recheck
							currentProcessedActorPtr->life = currentObject->life;
							currentProcessedActorPtr->lifeMode = currentObject->lifeMode;

							currentProcessedActorPtr->field_0 = i;

							setMoveMode(currentObject->trackMode, currentObject->trackNumber);

							currentProcessedActorPtr->positionInTrack = currentObject->positionInTrack;

							actorTurnedToObj = 1;
						}
					}
				} else {
					if(setupCameraSub1Sub1(currentObject->room))
						goto addObject;
				}
			}
		}

		currentObject++;
	}

	//  objModifFlag1 = 0;

	//TODO: object update
}

int checkActorInRoom(int room) {
	int i;

	for(i = 0; i < cameraDataTable[currentCamera]->_numCameraZoneDef; i++) {
		if(cameraDataTable[currentCamera]->_cameraZoneDefTable[i].dummy1 == room) {
			return(1);
		}
	}

	return(0);
}

void createActorList() {
	int i;
	actorStruct *actorPtr;

	numActorInList = 0;

	actorPtr = actorTable;

	for(i = 0; i < NUM_MAX_ACTOR; i++) {
		if(actorPtr->field_0 != -1 && actorPtr->bodyNum != -1) {
			if(checkActorInRoom(actorPtr->room)) {
				sortedActorTable[numActorInList] = i;
				if(!(actorPtr->flags & 0x21)) {
					actorPtr->flags |= 8;
					//  objModifFlag2 = 1;
				}
				numActorInList++;
			}
		}

		actorPtr++;
	}
}

void setupCamera() {
	int x;
	int y;
	int z;
	cameraDataStruct *pCamera;

	freezeTime();

	currentCamera = startGameVar1;

	loadCamera(roomDataTable[currentDisplayedRoom].cameraIdxTable[startGameVar1]);

	pCamera = cameraDataTable[currentCamera];

	setupPointTransformSM(pCamera->_alpha, pCamera->_beta, pCamera->_gamma);

#if INTERNAL_DEBUGGER
	if(debuggerVar_topCamera)
		setupPointTransformSM(0x100, 0, 0);
#endif

	x = (pCamera->_x - roomDataTable[currentDisplayedRoom].worldX) * 10;
	y = (roomDataTable[currentDisplayedRoom].worldY - pCamera->_y) * 10;
	z = (roomDataTable[currentDisplayedRoom].worldZ - pCamera->_z) * 10;

#if INTERNAL_DEBUGGER
	if(debuggerVar_topCamera) {
		x = actorTable[genVar9].worldX + actorTable[genVar9].modX;
		y = debufferVar_topCameraZoom;
		z = actorTable[genVar9].worldZ + actorTable[genVar9].modZ;
	}
#endif
	setupSelfModifyingCode(x, y, z); // setup camera position

	setupSMCode(160, 100, pCamera->_focal1, pCamera->_focal2, pCamera->_focal3); // setup focale

#if INTERNAL_DEBUGGER
	if(debuggerVar_topCamera)
		setupSMCode(160, 100, 1000, 100, 100); // setup focale
#endif

	setupCameraSub1();
	updateAllActorAndObjects();
	createActorList();
	//  setupCameraSub3();
	setupCameraSub4();
	/*  setupCameraSub5();
	 */
	if(mainVar1 == 2) {
		setupCameraVar1 = 2;
	} else {
		if(setupCameraVar1 != 2) {
			setupCameraVar1 = 1;
		}
	}

	mainVar1 = 0;
	unfreezeTime();
}

int16 computeDistanceToPoint(int x1, int z1, int x2, int z2) {
	int axBackup = x1;
	x1 -= x2;
	if((short int)x1 < 0) {
		x1 = -(short int)x1;
	}

	z1 -= z2;
	if((short int)z1 < 0) {
		z1 = -(short int)z1;
	}

	if((x1 + z1) > 0xFFFF) {
		return(0x7D00);
	} else {
		return(x1 + z1);
	}
}

void startActorRotation(int16 beta, int16 newBeta, int16 param, rotateStruct *rotatePtr) {
	rotatePtr->oldAngle = beta;
	rotatePtr->newAngle = newBeta;
	rotatePtr->param = param;
	rotatePtr->timeOfRotate = g_fitd->getTimer();
}

int16 updateActorRotation(rotateStruct *rotatePtr) {
	int timeDif;
	int angleDif;

	if(!rotatePtr->param)
		return(rotatePtr->newAngle);

	timeDif = g_fitd->getTimer() - rotatePtr->timeOfRotate;

	if(timeDif > rotatePtr->param) {
		rotatePtr->param = 0;
		return(rotatePtr->newAngle);
	}

	angleDif = (rotatePtr->newAngle & 0x3FF) - (rotatePtr->oldAngle & 0x3FF);

	if(angleDif <= 0x200) {
		if(angleDif >= -0x200) {
			int angle = (rotatePtr->newAngle & 0x3FF) - (rotatePtr->oldAngle & 0x3FF);
			return (rotatePtr->oldAngle & 0x3FF) + (angle * timeDif) / rotatePtr->param;
		} else {
			int16 angle = ((rotatePtr->newAngle & 0x3FF) + 0x400) - ((rotatePtr->oldAngle & 0x3FF));
			return (((rotatePtr->oldAngle & 0x3FF)) + ((angle * timeDif) / rotatePtr->param));
		}
	} else {
		int angle = (rotatePtr->newAngle & 0x3FF) - ((rotatePtr->oldAngle & 0x3FF) + 0x400);
		return ((angle * timeDif) / rotatePtr->param) + ((rotatePtr->oldAngle & 0x3FF));
	}
}

void deleteSub(int actorIdx) {
	actorStruct *actorPtr = &actorTable[actorIdx];

	actorPtr->flags &= 0xFFF7;

	//  objModifFlag2 = 1;

	BBox3D1 = actorPtr->field_14;

	if(BBox3D1 > -1) {
		BBox3D2 = actorPtr->field_16;
		BBox3D3 = actorPtr->field_18;
		BBox3D4 = actorPtr->field_1A;

		//deleteSubSub();
	}
}

void deleteObject(int objIdx) {
	objectStruct *objPtr;
	int actorIdx;
	actorStruct *actorPtr;

	objPtr = &objectTable[objIdx];
	actorIdx = objPtr->ownerIdx;

	if(actorIdx != -1) {
		actorPtr = &actorTable[actorIdx];

		actorPtr->room = -1;
		actorPtr->stage = -1;

		//    objModifFlag1 = 1;

		if(actorPtr->flags & 8) {
			deleteSub(actorIdx);
		}
	}

	objPtr->room = -1;
	objPtr->stage = -1;

	removeObjFromInventory(objIdx);
}

#ifdef INTERNAL_DEBUGGER
void line(int x1, int y1, int x2, int y2, char c);

#ifdef USE_GL
void drawProjectedLine(int32 x1s, int32 y1s, int32 z1s, int32 x2s, int32 y2s, int32 z2s, int c)
#else
void drawProjectedLine(int x1, int y1, int z1, int x2, int y2, int z2, int c)
#endif
{
#ifdef USE_GL
	float x1 = (float)x1s;
	float x2 = (float)x2s;
	float y1 = (float)y1s;
	float y2 = (float)y2s;
	float z1 = (float)z1s;
	float z2 = (float)z2s;

	float transformedX1;
	float transformedX2;

	float transformedY1;
	float transformedY2;
#else
	int transformedX1;
	int transformedX2;

	int transformedY1;
	int transformedY2;
#endif


	x1 -= translateX;
	x2 -= translateX;

	y1 -= translateY;
	y2 -= translateY;

	z1 -= translateZ;
	z2 -= translateZ;

	transformPoint(&x1, &y1, &z1);
	transformPoint(&x2, &y2, &z2);

	z1 += cameraX;
	z2 += cameraX;

#ifdef USE_GL
	transformedX1 = ((x1 * cameraY) / (float)z1) + cameraCenterX;
	transformedX2 = ((x2 * cameraY) / (float)z2) + cameraCenterX;

	transformedY1 = ((y1 * cameraZ) / (float)z1) + cameraCenterY;
	transformedY2 = ((y2 * cameraZ) / (float)z2) + cameraCenterY;
#else
	transformedX1 = ((x1 * cameraY) / z1) + cameraCenterX;
	transformedX2 = ((x2 * cameraY) / z2) + cameraCenterX;

	transformedY1 = ((y1 * cameraZ) / z1) + cameraCenterY;
	transformedY2 = ((y2 * cameraZ) / z2) + cameraCenterY;
#endif

#ifdef USE_GL
	if(z1 > 0 && z2 > 0)
		g_driver->draw3dLine(transformedX1, transformedY1, z1, transformedX2, transformedY2, z2, c);
#else
	if(z1 > 0 && z2 > 0)
		line(transformedX1, transformedY1, transformedX2, transformedY2, c);
#endif
}

void drawZv(actorStruct *actorPtr) {
	ZVStruct localZv;

	if(actorPtr->room != actorTable[genVar9].room) {
		getZvRelativePosition(&localZv, actorPtr->room, actorTable[genVar9].room);
	} else {
		copyZv(&actorPtr->zv, &localZv);
	}


	// bottom
	drawProjectedLine(localZv.ZVX1,
	                  localZv.ZVY2,
	                  localZv.ZVZ1,
	                  localZv.ZVX1,
	                  localZv.ZVY2,
	                  localZv.ZVZ2,
	                  10);

	drawProjectedLine(localZv.ZVX1, localZv.ZVY2, localZv.ZVZ2, localZv.ZVX2, localZv.ZVY2, localZv.ZVZ2, 10);
	drawProjectedLine(localZv.ZVX2, localZv.ZVY2, localZv.ZVZ2, localZv.ZVX2, localZv.ZVY2, localZv.ZVZ1, 10);
	drawProjectedLine(localZv.ZVX2, localZv.ZVY2, localZv.ZVZ1, localZv.ZVX1, localZv.ZVY2, localZv.ZVZ1, 10);

	// top
	drawProjectedLine(localZv.ZVX1, localZv.ZVY1, localZv.ZVZ1, localZv.ZVX1, localZv.ZVY1, localZv.ZVZ2, 10);
	drawProjectedLine(localZv.ZVX1, localZv.ZVY1, localZv.ZVZ2, localZv.ZVX2, localZv.ZVY1, localZv.ZVZ2, 10);
	drawProjectedLine(localZv.ZVX2, localZv.ZVY1, localZv.ZVZ2, localZv.ZVX2, localZv.ZVY1, localZv.ZVZ1, 10);
	drawProjectedLine(localZv.ZVX2, localZv.ZVY1, localZv.ZVZ1, localZv.ZVX1, localZv.ZVY1, localZv.ZVZ1, 10);

	drawProjectedLine(localZv.ZVX1, localZv.ZVY2, localZv.ZVZ1, localZv.ZVX1, localZv.ZVY1, localZv.ZVZ1, 10);
	drawProjectedLine(localZv.ZVX1, localZv.ZVY2, localZv.ZVZ2, localZv.ZVX1, localZv.ZVY1, localZv.ZVZ2, 10);
	drawProjectedLine(localZv.ZVX2, localZv.ZVY2, localZv.ZVZ2, localZv.ZVX2, localZv.ZVY1, localZv.ZVZ2, 10);
	drawProjectedLine(localZv.ZVX2, localZv.ZVY2, localZv.ZVZ1, localZv.ZVX2, localZv.ZVY1, localZv.ZVZ1, 10);


}
#endif

#ifdef INTERNAL_DEBUGGER
void drawConverZone(cameraZoneEntryStruct *zonePtr) {
	int i;

	for(i = 0; i < zonePtr->numPoints - 1; i++) {
		drawProjectedLine(zonePtr->pointTable[i].x * 10, 0, zonePtr->pointTable[i].y * 10, zonePtr->pointTable[i+1].x * 10, 0, zonePtr->pointTable[i+1].y * 10, 20);
	}

	// loop first and last

	i = zonePtr->numPoints - 1;
	drawProjectedLine(zonePtr->pointTable[0].x * 10, 0, zonePtr->pointTable[0].y * 10, zonePtr->pointTable[i].x * 10, 0, zonePtr->pointTable[i].y * 10, 20);
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawConverZones() {
	int i;
	for(i = 0; i < numCameraInRoom; i++) {
		int j;
		for(j = 0; j < cameraDataTable[i]->_numCameraZoneDef; j++) {
			int k;

			if(cameraDataTable[i]->_cameraZoneDefTable[j].dummy1 == currentDisplayedRoom) {
				for(k = 0; k < cameraDataTable[i]->_cameraZoneDefTable[j].numZones; k++) {
					drawConverZone(&cameraDataTable[i]->_cameraZoneDefTable[j].cameraZoneEntryTable[k]);
				}
			}
		}
	}
}
#endif

#ifdef USE_GL

#define DEPTH_THRESHOLD 1000

#ifdef INTERNAL_DEBUGGER
void drawProjectedQuad(float x1, float x2, float x3, float x4, float y1, float y2, float y3, float y4, float z1, float z2, float z3, float z4, int color, int transprency) {
	float transformedX1;
	float transformedX2;
	float transformedX3;
	float transformedX4;

	float transformedY1;
	float transformedY2;
	float transformedY3;
	float transformedY4;

	x1 -= translateX;
	x2 -= translateX;
	x3 -= translateX;
	x4 -= translateX;

	y1 -= translateY;
	y2 -= translateY;
	y3 -= translateY;
	y4 -= translateY;

	z1 -= translateZ;
	z2 -= translateZ;
	z3 -= translateZ;
	z4 -= translateZ;

	transformPoint(&x1, &y1, &z1);
	transformPoint(&x2, &y2, &z2);
	transformPoint(&x3, &y3, &z3);
	transformPoint(&x4, &y4, &z4);

	z1 += cameraX;
	z2 += cameraX;
	z3 += cameraX;
	z4 += cameraX;

	transformedX1 = ((x1 * cameraY) / (float)z1) + cameraCenterX;
	transformedX2 = ((x2 * cameraY) / (float)z2) + cameraCenterX;
	transformedX3 = ((x3 * cameraY) / (float)z3) + cameraCenterX;
	transformedX4 = ((x4 * cameraY) / (float)z4) + cameraCenterX;

	transformedY1 = ((y1 * cameraZ) / (float)z1) + cameraCenterY;
	transformedY2 = ((y2 * cameraZ) / (float)z2) + cameraCenterY;
	transformedY3 = ((y3 * cameraZ) / (float)z3) + cameraCenterY;
	transformedY4 = ((y4 * cameraZ) / (float)z4) + cameraCenterY;

	if(z1 > DEPTH_THRESHOLD && z2 > DEPTH_THRESHOLD && z3 > DEPTH_THRESHOLD && z4 > DEPTH_THRESHOLD) {
		g_driver->draw3dQuad(transformedX1, transformedY1, z1, transformedX2, transformedY2, z2, transformedX3, transformedY3, z3, transformedX4, transformedY4, z4, color, transprency);
	}

	//g_driver->draw3dQuad(x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4, color);
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawProjectedBox(int x1, int x2, int y1, int y2, int z1, int z2, int color, int transprency) {
	//bottom
	drawProjectedQuad((float)x1, (float)x1, (float)x2, (float)x2, (float)y1, (float)y1, (float)y1, (float)y1, (float)z1, (float)z2, (float)z2, (float)z1, color, transprency);
	//top
	drawProjectedQuad((float)x1, (float)x1, (float)x2, (float)x2, (float)y2, (float)y2, (float)y2, (float)y2, (float)z1, (float)z2, (float)z2, (float)z1, color, transprency);
	//left
	drawProjectedQuad((float)x1, (float)x1, (float)x1, (float)x1, (float)y1, (float)y2, (float)y2, (float)y1, (float)z1, (float)z1, (float)z2, (float)z2, color, transprency);
	//right
	drawProjectedQuad((float)x2, (float)x2, (float)x2, (float)x2, (float)y1, (float)y2, (float)y2, (float)y1, (float)z1, (float)z1, (float)z2, (float)z2, color, transprency);
	//front
	drawProjectedQuad((float)x1, (float)x2, (float)x2, (float)x1, (float)y1, (float)y1, (float)y2, (float)y2, (float)z1, (float)z1, (float)z1, (float)z1, color, transprency);
	//back
	drawProjectedQuad((float)x1, (float)x2, (float)x2, (float)x1, (float)y1, (float)y1, (float)y2, (float)y2, (float)z2, (float)z2, (float)z2, (float)z2, color, transprency);
}
#endif
#endif

#ifdef INTERNAL_DEBUGGER
void drawRoomZv(ZVStruct *zoneData, int color, int transparency) {
	ZVStruct cameraZv = { -100, 100, -100, 100, -100, 100};

	cameraZv.ZVX1 += translateX;
	cameraZv.ZVX2 += translateX;

	cameraZv.ZVY1 += translateY;
	cameraZv.ZVY2 += translateY;

	cameraZv.ZVZ1 += translateZ;
	cameraZv.ZVZ2 += translateZ;

	if(checkZvCollision(&cameraZv, zoneData)) {
		return;
	}

#ifdef USE_GL
	drawProjectedBox(zoneData->ZVX1, zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVY2, zoneData->ZVZ1, zoneData->ZVZ2, color, transparency);
#else
	drawProjectedLine(x1, y1, z1, x1, y1, z2, color);
	drawProjectedLine(x1, y1, z2, x2, y1, z2, color);
	drawProjectedLine(x2, y1, z2, x2, y1, z1, color);
	drawProjectedLine(x2, y1, z1, x1, y1, z1, color);

	drawProjectedLine(x1, y2, z1, x1, y2, z2, color);
	drawProjectedLine(x1, y2, z2, x2, y2, z2, color);
	drawProjectedLine(x2, y2, z2, x2, y2, z1, color);
	drawProjectedLine(x2, y2, z1, x1, y2, z1, color);

	drawProjectedLine(x1, y1, z1, x1, y2, z1, color);
	drawProjectedLine(x1, y1, z2, x1, y2, z2, color);
	drawProjectedLine(x2, y1, z2, x2, y2, z2, color);
	drawProjectedLine(x2, y1, z1, x2, y2, z1, color);
#endif
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawRoomZvLine(ZVStruct *zoneData, int color) {
	ZVStruct cameraZv = { -100, 100, -100, 100, -100, 100};

	cameraZv.ZVX1 += translateX;
	cameraZv.ZVX2 += translateX;

	cameraZv.ZVY1 += translateY;
	cameraZv.ZVY2 += translateY;

	cameraZv.ZVZ1 += translateZ;
	cameraZv.ZVZ2 += translateZ;

	if(checkZvCollision(&cameraZv, zoneData)) {
		return;
	}

	drawProjectedLine(zoneData->ZVX1, zoneData->ZVY1, zoneData->ZVZ1, zoneData->ZVX1, zoneData->ZVY1, zoneData->ZVZ2, color);
	drawProjectedLine(zoneData->ZVX1, zoneData->ZVY1, zoneData->ZVZ2, zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVZ2, color);
	drawProjectedLine(zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVZ2, zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVZ1, color);
	drawProjectedLine(zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVZ1, zoneData->ZVX1, zoneData->ZVY1, zoneData->ZVZ1, color);

	drawProjectedLine(zoneData->ZVX1, zoneData->ZVY2, zoneData->ZVZ1, zoneData->ZVX1, zoneData->ZVY2, zoneData->ZVZ2, color);
	drawProjectedLine(zoneData->ZVX1, zoneData->ZVY2, zoneData->ZVZ2, zoneData->ZVX2, zoneData->ZVY2, zoneData->ZVZ2, color);
	drawProjectedLine(zoneData->ZVX2, zoneData->ZVY2, zoneData->ZVZ2, zoneData->ZVX2, zoneData->ZVY2, zoneData->ZVZ1, color);
	drawProjectedLine(zoneData->ZVX2, zoneData->ZVY2, zoneData->ZVZ1, zoneData->ZVX1, zoneData->ZVY2, zoneData->ZVZ1, color);

	drawProjectedLine(zoneData->ZVX1, zoneData->ZVY1, zoneData->ZVZ1, zoneData->ZVX1, zoneData->ZVY2, zoneData->ZVZ1, color);
	drawProjectedLine(zoneData->ZVX1, zoneData->ZVY1, zoneData->ZVZ2, zoneData->ZVX1, zoneData->ZVY2, zoneData->ZVZ2, color);
	drawProjectedLine(zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVZ2, zoneData->ZVX2, zoneData->ZVY2, zoneData->ZVZ2, color);
	drawProjectedLine(zoneData->ZVX2, zoneData->ZVY1, zoneData->ZVZ1, zoneData->ZVX2, zoneData->ZVY2, zoneData->ZVZ1, color);
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawZone(char *zoneData, int color) {
	int x1;
	int x2;

	int y1;
	int y2;

	int z1;
	int z2;

	int type;

	ZVStruct tempZv;

	ZVStruct cameraZv = { -100, 100, -100, 100, -100, 100};

	type = *(int16 *)(zoneData + 0xE);

	x1 = *(int16 *)(zoneData + 0x0);
	x2 = *(int16 *)(zoneData + 0x2);
	y1 = *(int16 *)(zoneData + 0x4);
	y2 = *(int16 *)(zoneData + 0x6);
	z1 = *(int16 *)(zoneData + 0x8);
	z2 = *(int16 *)(zoneData + 0xA);

	cameraZv.ZVX1 += translateX;
	cameraZv.ZVX2 += translateX;

	cameraZv.ZVY1 += translateY;
	cameraZv.ZVY2 += translateY;

	cameraZv.ZVZ1 += translateZ;
	cameraZv.ZVZ2 += translateZ;

	tempZv.ZVX1 = (int16)READ_LE_UINT16(zoneData + 0x00);
	tempZv.ZVX2 = (int16)READ_LE_UINT16(zoneData + 0x02);
	tempZv.ZVY1 = (int16)READ_LE_UINT16(zoneData + 0x04);
	tempZv.ZVY2 = (int16)READ_LE_UINT16(zoneData + 0x06);
	tempZv.ZVZ1 = (int16)READ_LE_UINT16(zoneData + 0x08);
	tempZv.ZVZ2 = (int16)READ_LE_UINT16(zoneData + 0x0A);

	if(checkZvCollision(&cameraZv, &tempZv)) {
		return;
	}

#ifdef USE_GL
	drawProjectedBox(x1, x2, y1, y2, z1, z2, type, 255);
#else
	drawProjectedLine(x1, y1, z1, x1, y1, z2, type);
	drawProjectedLine(x1, y1, z2, x2, y1, z2, type);
	drawProjectedLine(x2, y1, z2, x2, y1, z1, type);
	drawProjectedLine(x2, y1, z1, x1, y1, z1, type);

	drawProjectedLine(x1, y2, z1, x1, y2, z2, type);
	drawProjectedLine(x1, y2, z2, x2, y2, z2, type);
	drawProjectedLine(x2, y2, z2, x2, y2, z1, type);
	drawProjectedLine(x2, y2, z1, x1, y2, z1, type);

	drawProjectedLine(x1, y1, z1, x1, y2, z1, type);
	drawProjectedLine(x1, y1, z2, x1, y2, z2, type);
	drawProjectedLine(x2, y1, z2, x2, y2, z2, type);
	drawProjectedLine(x2, y1, z1, x2, y2, z1, type);
#endif
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawOverlayZone(char *zoneData, int color) {
	int x1;
	int x2;

	int y1;
	int y2;

	int z1;
	int z2;

	x1 = *(int16 *)(zoneData + 0x0) * 10;
	z1 = *(int16 *)(zoneData + 0x2) * 10;
	x2 = *(int16 *)(zoneData + 0x4) * 10;
	z2 = *(int16 *)(zoneData + 0x6) * 10;

	y1 = 0;
	y2 = 0;

#ifdef USE_GL
	drawProjectedBox(x1, x2, y1, y2, z1, z2, color, 255);
#else
	drawProjectedLine(x1, y1, z1, x1, y1, z2, color);
	drawProjectedLine(x1, y1, z2, x2, y1, z2, color);
	drawProjectedLine(x2, y1, z2, x2, y1, z1, color);
	drawProjectedLine(x2, y1, z1, x1, y1, z1, color);

	drawProjectedLine(x1, y2, z1, x1, y2, z2, color);
	drawProjectedLine(x1, y2, z2, x2, y2, z2, color);
	drawProjectedLine(x2, y2, z2, x2, y2, z1, color);
	drawProjectedLine(x2, y2, z1, x1, y2, z1, color);

	drawProjectedLine(x1, y1, z1, x1, y2, z1, color);
	drawProjectedLine(x1, y1, z2, x1, y2, z2, color);
	drawProjectedLine(x2, y1, z2, x2, y2, z2, color);
	drawProjectedLine(x2, y1, z1, x2, y2, z1, color);
#endif
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawSceZone(int roomNumber) {
	uint32 i;
	ZVStruct dataLocal;

	for(i = 0; i < roomDataTable[roomNumber].numSceZone; i++) {
		memcpy(&dataLocal, &roomDataTable[roomNumber].sceZoneTable[i].zv, sizeof(ZVStruct));
		if(roomNumber != currentDisplayedRoom) {
			getZvRelativePosition(&dataLocal, roomNumber, currentDisplayedRoom);
		}

		if(roomDataTable[roomNumber].sceZoneTable[i].parameter == 4)
			if(roomDataTable[roomNumber].sceZoneTable[i].type) {
				drawRoomZv(&dataLocal, 20, 40);
			}
	}
}
#endif

#ifdef INTERNAL_DEBUGGER
void drawHardCol(int roomNumber) {
	uint32 i;
	ZVStruct dataLocal;

	for(i = 0; i < roomDataTable[roomNumber].numHardCol; i++) {
		copyZv(&roomDataTable[roomNumber].hardColTable[i].zv, &dataLocal);

		if(roomNumber != currentDisplayedRoom) {
			getZvRelativePosition(&dataLocal, roomNumber, currentDisplayedRoom);
		}

		switch(roomDataTable[roomNumber].hardColTable[i].type) {
		case 0: // objects 1
			drawRoomZv(&dataLocal, 9, 150);
			break;
		case 1: // walls
			drawRoomZv(&dataLocal, 180, 255);
			break;
		case 9: // objects 2
			drawRoomZv(&dataLocal, 5, 255);
			break;
		default:
			drawRoomZv(&dataLocal, 40, 40);
			break;
		}

	}
}
#endif


int isBgOverlayRequired(int X1, int X2, int Z1, int Z2, char *data, int param) {
	int i;
	for(i = 0; i < param; i++) {
		////////////////////////////////////// DEBUG
		//  drawOverlayZone(data, 80);
		/////////////////////////////////////

		int zoneX1 = *(int16 *)(data);
		int zoneZ1 = *(int16 *)(data + 2);
		int zoneX2 = *(int16 *)(data + 4);
		int zoneZ2 = *(int16 *)(data + 6);

		if(X1 >= zoneX1 && Z1 >= zoneZ1 && X2 <= zoneX2 && Z2 <= zoneZ2) {
			return(1);
		}

		data += 0x8;
	}

	return(0);
}

void drawBgOverlaySub2(int size) {
	int bx = 32767;
	int bp = 32767;
	int cx = -32768;
	int dx = -32768;
	int16 *out;
	int tempBxPtr;
	int tempCxPtr;

	char *si;

	int i;
	int saveDx;
	int saveAx;

	char *tempBufferSE;

	int direction = 1;

	int16 *data = (int16 *)cameraBuffer;

	overlaySize1 = size;
	overlaySize2 = size;

	bgOverlayVar1 = 0;

	for(i = 0; i < size; i++) {
		int temp = data[0];

		if(temp < bx)
			bx = temp;
		if(temp > dx)
			dx = temp;

		temp = data[1];

		if(temp < bp)
			bp = temp;
		if(temp > cx)
			cx = temp;

		data += 2;
	}

	out = data;
	data = (int16 *)cameraBuffer;

	out[0] = data[0];
	out[1] = data[1];

	out += 4;
	data += 4;

	if(cx == bp) {
		return;
	}

	cameraBufferPtr = cameraBuffer;
	cameraBuffer2Ptr = cameraBuffer2;
	cameraBuffer3Ptr = cameraBuffer3;

	si = cameraBufferPtr;
	tempBxPtr = *(int16 *)si;
	si += 2;
	tempCxPtr = *(int16 *)si;
	si += 2;

	tempBufferSE = cameraBuffer4;

	direction = 1;

	g_driver->startBgPoly();

	do {
		int dx;
		int ax;

		saveDx = *(int16 *)si;
		si += 2;
		saveAx = *(int16 *)si;
		si += 2;

#ifdef INTERNAL_DEBUGGER
		/*  if(backgroundMode == backgroundModeEnum_2D)
		 g_driver->draw3dLine(tempBxPtr, tempCxPtr, 0, saveDx,saveAx, 0, 130); */
#endif
		g_driver->addBgPolyPoint(tempBxPtr, tempCxPtr);

		dx = saveDx;
		ax = saveAx;

		if(ax != tempCxPtr) {
			char *ptr1;
			if(tempCxPtr == bp || tempCxPtr == cx) {
				char *temp = cameraBuffer3Ptr;
				cameraBuffer3Ptr = tempBufferSE;
				tempBufferSE = temp;
			}

			if(tempBxPtr >= dx) {
				int temp;
				temp = tempBxPtr;
				tempBxPtr = dx;
				dx = temp;

				temp = tempCxPtr;
				tempCxPtr = ax;
				ax = temp;
			}

			ptr1 = tempBufferSE + tempCxPtr * 2;

			if(tempCxPtr > ax) {
				int temp;
				direction = -1;


				temp = tempCxPtr;
				tempCxPtr = ax;
				ax = temp;
			}

			{
				// stupid, need optimisation
				int temp;

				temp = tempCxPtr;
				tempCxPtr = ax;
				ax = temp;
			}

			tempCxPtr -= ax;

			*(int16 *)ptr1 = tempBxPtr;
			ptr1 += 2 * direction;

			ax = tempBxPtr;

			dx -= tempBxPtr;
			tempBxPtr = dx;
			dx = tempCxPtr;
			bp = tempCxPtr / 2;

			do {
				bp += tempBxPtr;

				do {
					if(bp < dx)
						break;

					bp -= dx;
					ax ++;
				} while(1);

				*(int16 *)ptr1 = ax;
				ptr1 += 2 * direction;
			} while(--tempCxPtr);

			direction = 1;
		}

		tempCxPtr = saveAx;
		tempBxPtr = saveDx;
	} while(--overlaySize1);

	g_driver->endBgPoly();
}

void drawBgOverlay(actorStruct *actorPtr) {
	char *data;
	char *data2;
	int numEntry;
	int i;
	int numOverlayZone;

	actorPtr->field_14 = BBox3D1;
	actorPtr->field_16 = BBox3D2;
	actorPtr->field_18 = BBox3D3;
	actorPtr->field_1A = BBox3D4;

	setClipSize(BBox3D1, BBox3D2, BBox3D3, BBox3D4);

	data = roomVar5[currentCamera] + 0x12;
	numEntry = *(int16 *)(data);
	data += 2;

	while(numEntry > 0) {
		if(actorPtr->room == *(int16 *)(data)) {
			break;
		}
		data += 12;
		numEntry--;
	}

	if(numEntry == 0)
		return;

	data += 2;

	data2 = roomVar5[currentCamera] + *(uint16 *)(data);
	data = data2;
	data += 2;

	numOverlayZone = *(int16 *)(data2);

	for(i = 0; i < numOverlayZone; i++) {
		int numOverlay;
		char *src = data2 + *(uint16 *)(data + 2);

		if(isBgOverlayRequired(actorPtr->zv.ZVX1 / 10, actorPtr->zv.ZVX2 / 10,
		                       actorPtr->zv.ZVZ1 / 10, actorPtr->zv.ZVZ2 / 10,
		                       data + 4,
		                       *(int16 *)(data))) {
			int j;
			numOverlay = *(int16 *)src;
			src += 2;

			for(j = 0; j < numOverlay; j++) {
				int param = *(int16 *)(src);
				src += 2;

				memcpy(cameraBuffer, src, param * 4);

				src += param * 4;

				drawBgOverlaySub2(param);
			}

			//      blitOverlay(src);
		}

		numOverlay = *(int16 *)(data);
		data += 2;
		data += ((numOverlay * 4) + 1) * 2;
	}

	setClipSize(0, 0, 319, 199);
}

void mainDrawSub2(int actorIdx) { // draw flow
	actorStruct *actorPtr = &actorTable[actorIdx];

	char *data = hqrUnk->printTextSub2(actorPtr->FRAME);

	// TODO: finish
}

void getHotPoint(int hotPointIdx, char *bodyPtr, point3dStruct *hotPoint) {
	int16 flag;

	flag = *(int16 *)bodyPtr;
	bodyPtr += 2;

	if(flag & 2) {
		int16 offset;
		bodyPtr += 12;

		offset = *(int16 *)bodyPtr;
		bodyPtr += 2;
		bodyPtr += offset;

		offset = *(int16 *)bodyPtr; // num points
		bodyPtr += 2;
		bodyPtr += offset * 6; // skip point buffer

		offset = *(int16 *)bodyPtr; // num bones
		bodyPtr += 2;
		bodyPtr += offset * 2; // skip bone buffer

		ASSERT(hotPointIdx < offset);

		if(hotPointIdx < offset) {
			int pointIdx;
			int16 *source;

			if(flag & 8) {
				bodyPtr += hotPointIdx * 0x18;
			} else {
				bodyPtr += hotPointIdx * 16;
			}

			pointIdx = *(int16 *)(bodyPtr + 4); // first point

			//ASSERT(pointIdx > 0 && pointIdx < 1200);

			source = (int16 *)(((char *)pointBuffer) + pointIdx);

			hotPoint->x = source[0];
			hotPoint->y = source[1];
			hotPoint->z = source[2];
		} else {
			hotPoint->x = 0;
			hotPoint->y = 0;
			hotPoint->z = 0;
		}
	} else {
		hotPoint->x = 0;
		hotPoint->y = 0;
		hotPoint->z = 0;
	}
}

void mainDraw(int mode) {
#ifdef USE_GL
	if(mode == 2)
		g_driver->CopyBlockPhys((unsigned char *)screen, 0, 0, 320, 200);
#endif


	if(mode == 0) {
		//restoreDirtyRects();
	} else {
		genVar5 = 0;
		copyToScreen(aux2, screen);
	}

#ifdef USE_GL
	g_driver->startFrame();
#endif

	setClipSize(0, 0, 319, 199);
	genVar6 = 0;

#ifdef USE_GL
	g_driver->cleanScreenKeepZBuffer();
#endif

#ifdef INTERNAL_DEBUGGER
	if(backgroundMode == backgroundModeEnum_3D) {
		for(int i = 0; i < getNumberOfRoom(); i++) {
			drawHardCol(i);
			drawSceZone(i);
		}

		drawConverZones();
	}
#endif



#ifdef USE_GL
	g_driver->startModelRender();
#endif

	for(int i = 0; i < numActorInList; i++) {
		int currentDrawActor = sortedActorTable[i];
		actorStruct *actorPtr;

		actorPtr = &actorTable[currentDrawActor];

		//if(actorPtr->flags & 0x25)
		{
			actorPtr->flags &= 0xFFFB;

			if(actorPtr->flags & 0x20) {
				mainDrawSub2(currentDrawActor);
			} else {
				char *bodyPtr = listBody->get(actorPtr->bodyNum);

				if(listBody->getVar1()) {
					//          initAnimInBody(actorPtr->FRAME, HQR_Get(listAnim, actorPtr->ANIM), bodyPtr);
				}

				renderModel(actorPtr->worldX + actorPtr->modX, actorPtr->worldY + actorPtr->modY, actorPtr->worldZ + actorPtr->modZ,
				            actorPtr->alpha, actorPtr->beta, actorPtr->gamma, bodyPtr);


				if(actorPtr->animActionType && actorPtr->field_98 != -1) {
					getHotPoint(actorPtr->field_98, bodyPtr, &actorPtr->hotPoint);
				}

				///////////////////////////////////// DEBUG
#ifdef INTERNAL_DEBUGGER
				//  if(debuggerVar_drawModelZv)
				{
					if(backgroundMode == backgroundModeEnum_3D) {
						drawZv(actorPtr);
					}
				}
#endif
				/////////////////////////////////////
			}

			if(BBox3D1 < 0)
				BBox3D1 = 0;
			if(BBox3D3 > 319)
				BBox3D3 = 319;
			if(BBox3D2 < 0)
				BBox3D2 = 0;
			if(BBox3D4 > 199)
				BBox3D4 = 199;

			if(BBox3D1 <= 319 && BBox3D2 <= 199 && BBox3D3 >= 0 && BBox3D4 >= 0) { // is the character on screen ?
				if(g_fitd->getGameType() == GType_AITD1) {
					if(actorPtr->field_0 == CVars[getCVarsIdx(LIGHT_OBJECT)]) {
						mainVar3 = (BBox3D3 + BBox3D1) / 2;
						mainVar2 = (BBox3D4 + BBox3D2) / 2;
					}
				}

#ifdef INTERNAL_DEBUGGER
				if(backgroundMode == backgroundModeEnum_2D)
#endif
				{
					if(g_fitd->getGameType() == GType_AITD1)
						drawBgOverlay(actorPtr);
				}
				//addToRedrawBox();
			} else {
				actorPtr->field_1A = -1;
				actorPtr->field_18 = -1;
				actorPtr->field_16 = -1;
				actorPtr->field_14 = -1;
			}
		}
	}

#ifdef USE_GL
	g_driver->stopModelRender();
#endif

	if(drawTextOverlay()) {
		//addToRedrawBox();
	}

	if(!lightVar1) {
		if(mode) {
			if(mode != 2 || lightVar2) {
				//makeBlackPalette();
				flipScreen();
				make3dTatouUnk1(0x10, 0);
				lightVar2 = 0;
			} else {
				//flipScreen();
			}
		} else {
			//mainDrawSub1();
		}
	} else {
	}

#ifdef INTERNAL_DEBUGGER
	debugger_draw();
#endif

#ifdef USE_GL
	g_driver->stopFrame();
#endif

	flipScreen();
}

void walkStep(int angle1, int angle2, int angle3) {
	makeRotationMtx(angle3, angle1, angle2, &animMoveY, &animMoveX);
}

void stopAnim(int actorIdx) {
	actorTable[actorIdx].flags |= 0xC;
	actorTable[actorIdx].flags &= 0xFFFE;

	//objModifFlag2 = 1;
}

int checkZvCollision(ZVStruct *zvPtr1, ZVStruct *zvPtr2) {
	if(zvPtr1->ZVX1 >= zvPtr2->ZVX2)
		return 0;

	if(zvPtr2->ZVX1 >= zvPtr1->ZVX2)
		return 0;

	if(zvPtr1->ZVY1 >= zvPtr2->ZVY2)
		return 0;

	if(zvPtr2->ZVY1 >= zvPtr1->ZVY2)
		return 0;

	if(zvPtr1->ZVZ1 >= zvPtr2->ZVZ2)
		return 0;

	if(zvPtr2->ZVZ1 >= zvPtr1->ZVZ2)
		return 0;

	return 1;
}

void getZvRelativePosition(ZVStruct *zvPtr, int startRoom, int destRoom) {
	unsigned int Xdif = 10 * (roomDataTable[destRoom].worldX - roomDataTable[startRoom].worldX);
	unsigned int Ydif = 10 * (roomDataTable[destRoom].worldY - roomDataTable[startRoom].worldY);
	unsigned int Zdif = 10 * (roomDataTable[destRoom].worldZ - roomDataTable[startRoom].worldZ);

	zvPtr->ZVX1 -= Xdif;
	zvPtr->ZVX2 -= Xdif;
	zvPtr->ZVY1 += Ydif;
	zvPtr->ZVY2 += Ydif;
	zvPtr->ZVZ1 += Zdif;
	zvPtr->ZVZ2 += Zdif;
}

void cleanFoundObjectScreen() {
	// 160 120 whidth = ,240,120
	memset(screen, 0, 320 * 200);
}

void drawFoundObect(int menuState, int objectName, int zoomFactor) {
	cleanFoundObjectScreen();

	rotateModel(0, 0, 0, 60, statusVar1, 0, zoomFactor);

	renderModel(0, 0, 0, 0, 0, 0, listBody->get(currentFoundBodyIdx));

	drawText(160, currentMenuTop, 20, 1);
	drawText(160, currentMenuTop + 16, objectName, 1);
	drawText(160, currentMenuTop + 16, objectName, 1);

	switch(menuState) {
	case 0: {
		drawSlectedText(130, currentMenuBottom - 16, 21, 1, 4);
		drawText(190, currentMenuBottom - 16, 22, 4);
		break;
	}
	case 1: {
		drawText(130, currentMenuBottom - 16, 21, 4);
		drawSlectedText(190, currentMenuBottom - 16, 22, 1, 4);
		break;
	}
	case 2: {
		drawSlectedText(160, currentMenuBottom - 16, 10, 1, 4);
		break;
	}
	}
}

void take(int objIdx) {
	objectStruct *objPtr = &objectTable[objIdx];

	if(numObjInInventory == 0) {
		inventory[0] = objIdx;
	} else {
		int i;

		for(i = numObjInInventory; i > 0; i--) {
			inventory[i+1] = inventory[i];
		}

		inventory[1] = objIdx;
	}

	numObjInInventory++;

	action = 0x800;

	updateInHand(objIdx);

	if(objPtr->ownerIdx != -1) {
		updateAllActorAndObjectsSub1(objPtr->ownerIdx);
	}

	objPtr->flags2 &= 0xBFFF;
	objPtr->flags2 |= 0x8000;

	objPtr->room = -1;
	objPtr->stage = -1;
}

void foundObject(int objIdx, int param) {
	objectStruct *objPtr;
	int var_C = 0;
	int var_6 = 1;
	int var_2 = 0;
	int i;
	int var_A = 15000;
	int var_8 = -200;

	if(objIdx < 0)
		return;

	objPtr = &objectTable[objIdx];

	if(param != 0 && (objPtr->flags2 & 0xC000)) {
		return;
	}

	if(objPtr->trackNumber) {
		if(g_fitd->getTimer() - objPtr->trackNumber < 300) // prevent from reopening the window every frame
			return;
	}

	objPtr->trackNumber = 0;

	freezeTime();
	//  setupShaking(1000); // probably to remove the shaking when in foundObject screen

	for(i = 0; i < numObjInInventory; i++) {
		var_2 += objectTable[inventory[i]].positionInTrack;
	}

	if(objPtr->positionInTrack + var_2 > CVars[getCVarsIdx(MAX_WEIGHT_LOADABLE)] || numObjInInventory + 1 == 30) {
		var_6 = 3;
	}

	currentFoundBodyIdx = objPtr->foundBody;
	currentFoundBody = listBody->get(currentFoundBodyIdx);

	setupSMCode(160, 100, 128, 300, 298);

	statusVar1 = 0;

	copyToScreen(unkScreenVar, screen);

	drawAITDBox(160, 100, 240, 120);

	drawFoundObect(var_6, objPtr->foundName, var_A);
	flipScreen();

	input5 = 1;

	while(!var_C) {
#ifdef USE_GL
		g_driver->CopyBlockPhys((unsigned char *)screen, 0, 0, 320, 200);
		g_driver->startFrame();
		g_driver->cleanScreenKeepZBuffer();
#endif

		process_events();
		readKeyboard();

		input3 = input2;
		input4 = inputKey;
		button = input1;

		if(!input5) {
			if(input3 == 1) {
				if(var_6 != 2) {
					var_6 = 0;
				}

				var_C = 1;
			}
			if(var_6 != 2) {
				if(input4 & 4) {
					var_6 = 0;
				}

				if(input4 & 8) {
					var_6 = 1;
				}
			}

			if(input3 == 28 || button != 0) {
				while(input2)
					readKeyboard();

				var_C = 1;
			}
		} else {
			if(!input3 && !input4 && !button)
				input5 = 0;
		}

		statusVar1 -= 8;

		var_A += var_8; // zoom / dezoom

		if(var_A > 8000) // zoom management
			var_8 = -var_8;

		if(var_A < 25000)
			var_8 = -var_8;

		drawFoundObect(var_6, objPtr->foundName, var_A);
		flipScreen();

		//    menuWaitVSync();
	}

	unfreezeTime();

	if(var_6 == 1) {
		take(objIdx);
	} else {
		objPtr->trackNumber = g_fitd->getTimer();
	}

	while(input2 && input1) {
		readKeyboard();
	}

	input4 = 0;
	input3 = 0;
	button = 0;

	//  if(mainLoopVar1 != 0)
	{
		//setupShaking(-600);
	}

	mainVar1 = 1;
}

int checkForHardCol(ZVStruct *zvPtr, roomDataStruct *pRoomData) {
	uint16 i;
	int hardColVar = 0;
	hardColStruct *pCurrentEntry = pRoomData->hardColTable;

#ifdef INTERNAL_DEBUGGER
	if(debuggerVar_noHardClip)
		return 0;
#endif

	for(i = 0; i < pRoomData->numHardCol; i++) {
		if(((pCurrentEntry->zv.ZVX1) < (zvPtr->ZVX2)) && ((zvPtr->ZVX1) < (pCurrentEntry->zv.ZVX2))) {
			if(((pCurrentEntry->zv.ZVY1) < (zvPtr->ZVY2)) && ((zvPtr->ZVY1) < (pCurrentEntry->zv.ZVY2))) {
				if(((pCurrentEntry->zv.ZVZ1) < (zvPtr->ZVZ2)) && ((zvPtr->ZVZ1) < (pCurrentEntry->zv.ZVZ2))) {
					ASSERT(hardColVar < 10);
					hardColTable[hardColVar++] = pCurrentEntry;
				}
			}
		}

		pCurrentEntry++;
	}

	return hardColVar;
}

void menuWaitVSync() {
}

int changeCameraSub1Sub1(int x1, int z1, int x2, int z2, int x3, int z3, int x4, int z4) {
	int returnFlag = 0;

	int var1 = x1 - x2;
	int var2 = z3 - z4;
	int var3 = x3 - x4;
	int var4 = z1 - z2;

	int var5 = x1 - x3;
	int var6 = z1 - z3;

	int result1 = (var1 * var2) - (var3 * var4);

	int result2;
	int result3;

	if(!result1) {
		return(returnFlag);
	}

	result2 = (var5 * var2) - (var3 * var6);

	result3 = (-var1 * var6) + (var5 * var4);

	if(result1 < 0) {
		result1 = -result1;
		result2 = -result2;
		result3 = -result3;
	}

	if(result2 <= 0 || result3 <= 0) {
		return(returnFlag);
	}

	if(result1 > result2 && result1 > result3) {
		returnFlag = 1;
	}

	return(returnFlag);
}

int changeCameraSub1(int x1, int x2, int z1, int z2, cameraZoneDefStruct *pCameraZoneDef) {
	int xMid = (x1 + x2) / 2;
	int zMid = (z1 + z2) / 2;

	for(int i = 0; i < pCameraZoneDef->numZones; i++) {
		int flag = 0;

		for(int j = 0; j < pCameraZoneDef->cameraZoneEntryTable[i].numPoints; j++) {
			int zoneX1;
			int zoneZ1;
			int zoneX2;
			int zoneZ2;

			zoneX1 = pCameraZoneDef->cameraZoneEntryTable[i].pointTable[j].x;
			zoneZ1 = pCameraZoneDef->cameraZoneEntryTable[i].pointTable[j].y;
			zoneX2 = pCameraZoneDef->cameraZoneEntryTable[i].pointTable[j+1].x;
			zoneZ2 = pCameraZoneDef->cameraZoneEntryTable[i].pointTable[j+1].y;

			if(changeCameraSub1Sub1(xMid, zMid, xMid - 10000, zMid, zoneX1, zoneZ1, zoneX2, zoneZ2)) {
				flag |= 1;
			}

			if(changeCameraSub1Sub1(xMid, zMid, xMid + 10000, zMid, zoneX1, zoneZ1, zoneX2, zoneZ2)) {
				flag |= 2;
			}
		}

		if(flag == 3) {
			return(1);
		}
	}

	return(0);
}

int changeCameraSub2(void) {
	int foundDistance = 32000;
	int foundCamera = -1;

	actorStruct *actorPtr = &actorTable[genVar9];

	int x1 = actorPtr->zv.ZVX1 / 10;
	int x2 = actorPtr->zv.ZVX2 / 10;
	int z1 = actorPtr->zv.ZVZ1 / 10;
	int z2 = actorPtr->zv.ZVZ2 / 10;

	for(int i = 0; i < numCameraInRoom; i++) {
		ASSERT(i < NUM_MAX_CAMERA_IN_ROOM);
		if(changeCameraSub1(x1, x2, z1, z2, currentCameraZoneList[i])) { // if in camera zone ?
			int newAngle = actorPtr->beta + (((cameraDataTable[i]->_beta) + 0x200) & 0x3FF);

			if(newAngle) {
				newAngle = -newAngle;
			}

			if(newAngle < foundDistance) {
				foundDistance = newAngle;
				foundCamera = i;
			}
		}
	}

	return(foundCamera);
}

void checkIfCameraChangeIsRequired(void) {
	int localCurrentCam = currentCamera;
	int newCamera;

	if(currentCamera != -1) {
		actorStruct *actorPtr;
		int zvx1;
		int zvx2;
		int zvz1;
		int zvz2;

		actorPtr = &actorTable[genVar9];

		zvx1 = actorPtr->zv.ZVX1 / 10;
		zvx2 = actorPtr->zv.ZVX2 / 10;

		zvz1 = actorPtr->zv.ZVZ1 / 10;
		zvz2 = actorPtr->zv.ZVZ2 / 10;

		if(changeCameraSub1(zvx1, zvx2, zvz1, zvz2, currentCameraZoneList[currentCamera])) { // is still in current camera zone ?
			return;
		}
	}

#ifdef INTERNAL_DEBUGGER
	//printf("Exited current camera cover zone...\n");
#endif

	newCamera = changeCameraSub2(); // find new camera

	if(newCamera != -1) {
		localCurrentCam = newCamera;
	}

	if(currentCamera != localCurrentCam) {
		startGameVar1 = localCurrentCam;
		mainVar1 = 1;
	}

#ifdef INTERNAL_DEBUGGER
	/* if(newCamera == -1)
	 {
	 printf("No new camera found...\n");
	 }*/
#endif
}


void putAt(int objIdx, int objIdxToPutAt) {
	objectStruct *objPtr = &objectTable[objIdx];
	objectStruct *objPtrToPutAt = &objectTable[objIdxToPutAt];

	if(objPtrToPutAt->ownerIdx != -1) {
		actorStruct *actorToPutAtPtr = &actorTable[objPtrToPutAt->ownerIdx];

		removeObjFromInventory(objIdx);

		if(objPtr->ownerIdx == -1) {
			objPtr->x = actorToPutAtPtr->roomX;
			objPtr->y = actorToPutAtPtr->roomY;
			objPtr->z = actorToPutAtPtr->roomZ;
			objPtr->room = actorToPutAtPtr->room;
			objPtr->stage = actorToPutAtPtr->stage;
			objPtr->alpha = actorToPutAtPtr->alpha;
			objPtr->beta = actorToPutAtPtr->beta;
			objPtr->gamma = actorToPutAtPtr->gamma;

			objPtr->flags2 |= 0x4000;
			objPtr->flags |= 0x80;

			//      objModifFlag1 = 1;
			//      objModifFlag2 = 1;
		} else {
			currentProcessedActorPtr->roomX = actorToPutAtPtr->roomX;
			currentProcessedActorPtr->roomY = actorToPutAtPtr->roomY;
			currentProcessedActorPtr->roomZ = actorToPutAtPtr->roomZ;
			currentProcessedActorPtr->room = actorToPutAtPtr->room;
			currentProcessedActorPtr->stage = actorToPutAtPtr->stage;
			currentProcessedActorPtr->alpha = actorToPutAtPtr->alpha;
			currentProcessedActorPtr->beta = actorToPutAtPtr->beta;
			currentProcessedActorPtr->gamma = actorToPutAtPtr->gamma;

			objectTable[currentProcessedActorPtr->field_0].flags2 |= 0x4000;
			objectTable[currentProcessedActorPtr->field_0].flags |= 0x80;

			//      objModifFlag1 = 1;
			//      objModifFlag2 = 1;
		}

	} else {
		removeObjFromInventory(objIdx);

		if(objPtr->ownerIdx == -1) {
			objPtr->x = objPtrToPutAt->x;
			objPtr->y = objPtrToPutAt->y;
			objPtr->z = objPtrToPutAt->z;
			objPtr->room = objPtrToPutAt->room;
			objPtr->stage = objPtrToPutAt->stage;
			objPtr->alpha = objPtrToPutAt->alpha;
			objPtr->beta = objPtrToPutAt->beta;
			objPtr->gamma = objPtrToPutAt->gamma;

			objPtr->flags2 |= 0x4000;
			objPtr->flags |= 0x80;

			//      objModifFlag1 = 1;
			//      objModifFlag2 = 1;
		} else {
			currentProcessedActorPtr->roomX = objPtrToPutAt->x;
			currentProcessedActorPtr->roomY = objPtrToPutAt->y;
			currentProcessedActorPtr->roomZ = objPtrToPutAt->z;
			currentProcessedActorPtr->room = objPtrToPutAt->room;
			currentProcessedActorPtr->stage = objPtrToPutAt->stage;
			currentProcessedActorPtr->alpha = objPtrToPutAt->alpha;
			currentProcessedActorPtr->beta = objPtrToPutAt->beta;
			currentProcessedActorPtr->gamma = objPtrToPutAt->gamma;

			objectTable[currentProcessedActorPtr->field_0].flags2 |= 0x4000;
			objectTable[currentProcessedActorPtr->field_0].flags |= 0x80;

			//      objModifFlag1 = 1;
			//      objModifFlag2 = 1;
		}
	}
}

void throwStoppedAt(int x, int z) {
	int x2;
	int y2;
	int z2;
	int foundPosition;
	int step;

	ZVStruct zvCopy;
	ZVStruct zvLocal;
	uint8 *bodyPtr;

	bodyPtr = (uint8 *)listBody->get(currentProcessedActorPtr->bodyNum);

	getZvNormal((char *)bodyPtr, &zvLocal);

	x2 = x;
	y2 = (currentProcessedActorPtr->roomY / 2000) * 2000;
	z2 = z;

	foundPosition = 0;
	step = 0;

	while(!foundPosition) {
		walkStep(0, -step, currentProcessedActorPtr->beta + 0x200);
		copyZv(&zvLocal, &zvCopy);

		x2 = x + animMoveX;
		z2 = z + animMoveY;

		zvCopy.ZVX1 += x2;
		zvCopy.ZVX2 += x2;

		zvCopy.ZVY1 += y2;
		zvCopy.ZVY2 += y2;

		zvCopy.ZVZ1 += z2;
		zvCopy.ZVZ2 += z2;

		if(!checkForHardCol(&zvCopy, &roomDataTable[currentProcessedActorPtr->room])) {
			foundPosition = 1;
		}

		if(foundPosition) {
			if(y2 < -500) {
				zvCopy.ZVY1 += 100; // is the object reachable ? (100 is Carnby height. If hard col at Y + 100, carnby can't reach that spot)
				zvCopy.ZVY2 += 100;

				if(!checkForHardCol(&zvCopy, &roomDataTable[currentProcessedActorPtr->room])) {
					y2 += 2000;
					foundPosition = 0;
				} else {
					zvCopy.ZVY1 -= 100;
					zvCopy.ZVY2 -= 100;
				}
			}
		} else {
			step += 100;
		}
	}

	currentProcessedActorPtr->worldX = x2;
	currentProcessedActorPtr->roomX = x2;
	currentProcessedActorPtr->worldY = y2;
	currentProcessedActorPtr->roomY = y2;
	currentProcessedActorPtr->worldZ = z2;
	currentProcessedActorPtr->roomZ = z2;

	currentProcessedActorPtr->modX = 0;
	currentProcessedActorPtr->modZ = 0;

	currentProcessedActorPtr->animActionType = 0;
	currentProcessedActorPtr->speed = 0;
	currentProcessedActorPtr->gamma = 0;

	getZvNormal((char *)bodyPtr, &currentProcessedActorPtr->zv);

	currentProcessedActorPtr->zv.ZVX1 += x2;
	currentProcessedActorPtr->zv.ZVX2 += x2;
	currentProcessedActorPtr->zv.ZVY1 += y2;
	currentProcessedActorPtr->zv.ZVY2 += y2;
	currentProcessedActorPtr->zv.ZVZ1 += z2;
	currentProcessedActorPtr->zv.ZVZ2 += z2;

	objectTable[currentProcessedActorPtr->field_0].flags2 |= 0x4000;
	objectTable[currentProcessedActorPtr->field_0].flags2 &= 0xEFFF;

	stopAnim(currentProcessedActorIdx);
}

void startGame(int startupFloor, int startupRoom, int allowSystemMenu) {
	initEngine();
	initVars();

	loadFloor(startupFloor);

	currentCamera = -1;

	loadRoom(startupRoom);

	startGameVar1 = 0;
	mainVar1 = 2;

	setupCamera();

	mainLoop(allowSystemMenu);

	/*freeScene();

	 fadeOut(8,0);*/
}

int parseAllSaves(int arg) {
	return(0);
	// TODO : make real implementation
}

void configureHqrHero(hqrEntryStruct *hqrPtr, const char *name) {
	char temp[10];
	strcpy(temp, "        ");
	strncpy(temp, name, 8);
	hqrPtr->setString(temp);
}

int drawTextOverlay(void) {
	int var_2 = 0;
	int var_14 = 0;
	int var_10 = 183;
	messageStruct *currentMessage;

	BBox3D4 = 199;
	BBox3D1 = 319;
	BBox3D3 = 0;

	currentMessage = messageTable;

	if(lightVar1 == 0) {
		for(int i = 0; i < 5; i++) {
			if(currentMessage->string) {
				int width = currentMessage->string->width;
				int X = 160 - width / 2;
				int Y = X + width;

				if(X < BBox3D1) {
					BBox3D1 = X;
				}

				if(Y > BBox3D3) {
					BBox3D3 = Y;
				}

				if((currentMessage->time++) > 55) {
					currentMessage->string = NULL;
				} else {
					if(currentMessage->time < 26) {
						initFont(fontData, 16);
					} else {
						initFont(fontData, 16 + (currentMessage->time - 26) / 2);
					}

					renderText(X, var_10 + 1, screen, currentMessage->string->textPtr);
				}

				var_10 -= 16;
				var_14 = 1;

			}

			currentMessage++;
		}
	} else {
	}

	BBox3D2 = var_10;
	return(var_14);
}

void makeMessage(int messageIdx) {
	textEntryStruct *messagePtr;

	messagePtr = getTextFromIdx(messageIdx);

	if(messagePtr) {
		for(int i = 0; i < 5; i++) {
			if(messageTable[i].string == messagePtr) {
				messageTable[i].time = 0;
				return;
			}
		}

		for(int i = 0; i < 5; i++) {
			if(messageTable[i].string == NULL) {
				messageTable[i].string = messagePtr;
				messageTable[i].time = 0;
				return;
			}
		}
	}
}

void hit(int animNumber, int arg_2, int arg_4, int arg_6, int hitForce, int arg_A) {
	if(anim(animNumber, 0, arg_A)) {
		currentProcessedActorPtr->animActionANIM = animNumber;
		currentProcessedActorPtr->animActionFRAME = arg_2;
		currentProcessedActorPtr->animActionType = 1;
		currentProcessedActorPtr->animActionParam = arg_6;
		currentProcessedActorPtr->field_98 = arg_4;
		currentProcessedActorPtr->hitForce = hitForce;
	}
}

void setClipSize(int left, int top, int right, int bottom) {
	clipLeft = left;
	clipTop = top;
	clipRight = right;
	clipBottom = bottom;
}

void Sound_Quit(void);

void cleanupAndExit(void) {
	Sound_Quit();

	delete hqrUnk;
	delete listLife;
	delete listTrack;
	delete listBody;
	delete listAnim;

	/* free(tabTextes);
	 free(aux);
	 free(aux2);
	 free(bufferAnim);

	 free(screen); */

	destroyMusicDriver();

	error("Exiting");
}

} // end of namespace Fitd

using namespace Fitd;

int main(int argc, char **argv) {
	//  int protectionToBeDone = 1;
	char version[256];

	getVersion(version);

	warning(version);

	g_fitd = new FitdEngine();

	startThreadTimer();

	g_fitd->run();

	return(0);
}

