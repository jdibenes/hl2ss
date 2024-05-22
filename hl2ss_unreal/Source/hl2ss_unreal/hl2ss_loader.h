// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "hl2ss_loader.generated.h"

UCLASS()
class HL2SS_UNREAL_API Ahl2ss_loader : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	Ahl2ss_loader();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
