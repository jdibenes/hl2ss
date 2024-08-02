// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "hl2ss_loader.generated.h"

UCLASS()
class HL2SS_UNREAL_API Ahl2ss_loader : public AActor
{
	GENERATED_BODY()
	
	wchar_t const* const MUTEX_PV = L"x38.hl2ss.pv.mutex";
	wchar_t const* const MUTEX_EV = L"x38.hl2ss.ev.mutex";

	int mqx_state;
	HANDLE pv_mutex;
	HANDLE ev_mutex;

public:	
	// Sets default values for this actor's properties
	Ahl2ss_loader();

	void ProcessClientMessage();
	void ProcessServerMessage();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
