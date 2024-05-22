// Fill out your copyright notice in the Description page of Project Settings.


#include "hl2ss_loader.h"
#include "hl2ss_api.h"

// Sets default values
Ahl2ss_loader::Ahl2ss_loader()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void Ahl2ss_loader::BeginPlay()
{
	Super::BeginPlay();
	
	// Load HL2SS library and start streams
	hl2ss_api::Initialize();
	hl2ss_api::InitializeStreams(hl2ss_api::ENABLE_ALL);

	// TODO: Link coordinate systems
	hl2ss_api::OverrideWorldCoordinateSystem(NULL);

	// Test functions
	wchar_t buffer[16];
	memset(buffer, 0, sizeof(buffer));
	hl2ss_api::GetLocalIPv4Address(buffer, sizeof(buffer));
	auto ip_address = FString(buffer);
	hl2ss_api::DebugMessage(StringCast<ANSICHAR>(*ip_address).Get());

}

// Called every frame
void Ahl2ss_loader::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Translate and process client messages

	uint32_t size = hl2ss_api::MQ_SI_Peek();
	if (size == hl2ss_api::QUEUE_EMPTY) { return; }

	uint32_t command;
	uint8_t* data = new uint8_t[size + 1]; // delete
	hl2ss_api::MQ_SI_Pop(command, data);
	data[size] = '\0';

	uint32_t response;

	switch (command)
	{
	case 0xFFFFFFFE:
		hl2ss_api::DebugMessage((char*)data);
		response = 1;
		break;
	default:
		response = 0;
	}

	delete[] data;

	hl2ss_api::MQ_SO_Push(response);

	if (command == hl2ss_api::CLIENT_DISCONNECTED) { hl2ss_api::MQ_Restart(); }
}

