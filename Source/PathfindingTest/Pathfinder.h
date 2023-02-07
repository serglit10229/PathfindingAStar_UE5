// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include <vector>
#include "Kismet/KismetSystemLibrary.h"
#include "Algo/Reverse.h"
#include <queue>
#include <functional>
#include "DrawDebugHelpers.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"

#include "Pathfinder.generated.h"






UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class PATHFINDINGTEST_API UPathfinder : public USceneComponent
{
	GENERATED_BODY()




public:

	struct Node
	{
		FVector worldLocation;
		bool walkable;

		int gridX;
		int gridY;
		float gCost = -1;
		float hCost = -1;

		Node* parent = nullptr;


		Node(FVector location, bool walkable, int x, int y)
		{
			this->worldLocation = location;
			this->walkable = walkable;
			gridX = x;
			gridY = y;
		}

		float fCost()
		{
			return gCost + hCost;
		}

		bool operator<(Node& other) 
		{
			if (fCost() == other.fCost())
			{
				return hCost < other.hCost;
			}

			return fCost() > other.fCost();
		}
	};

	UPROPERTY(EditAnywhere)
		FVector2D NavigationPlaneExtent = FVector2D(1000.0f, 1000.0f);

	UPROPERTY(EditAnywhere)
		float gridSize = 10.0f;

	UPROPERTY(EditAnywhere)
		int NumberOfObstacles = 100;

	UPROPERTY(Category = Private, VisibleDefaultsOnly)
		UHierarchicalInstancedStaticMeshComponent* ISM_Obstacles;

	FVector gridStartLocation;
	Node** allNodes;
	int allNodesSize;
	int gridLength;
	int gridWidth;

	TArray<Node*> path;
	FTimerHandle AnimationTimerHandle;


public:
	UFUNCTION(BlueprintCallable)
		bool FindPath(FVector start, FVector finish);

	UFUNCTION(BlueprintCallable)
		void InitObstacles();

	UFUNCTION(BlueprintCallable)
		void InitNavGrid();

private:

	Node* NodeFromLocation(FVector location);
	bool CheckNodeWalkable(FVector worldLocation);
	TArray<Node*> GetNeighbours(Node* node);
	int GetDistance(Node* a, Node* b);
	TArray<Node*> RetracePath(Node* startNode, Node* finishNode);
	void DrawPathSphere(int* i);


public:
	UPathfinder();
protected:
	virtual void BeginPlay() override;
public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
