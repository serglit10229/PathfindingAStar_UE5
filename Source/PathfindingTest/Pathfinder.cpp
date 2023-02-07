// Fill out your copyright notice in the Description page of Project Settings.


#include "Pathfinder.h"

// Sets default values for this component's properties
UPathfinder::UPathfinder()
{
	PrimaryComponentTick.bCanEverTick = true;

	this->ISM_Obstacles = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("HierarchicalInstancedStaticMesh"));
	//ISM_Obstacles->SetupAttachment(this);

	static ConstructorHelpers::FObjectFinder<UStaticMesh>CubeMeshAsset(TEXT("StaticMesh'/Engine/BasicShapes/Cube.Cube'"));
	UStaticMesh* CubeAsset = CubeMeshAsset.Object;
	ISM_Obstacles->SetStaticMesh(CubeAsset);
	ISM_Obstacles->SetCollisionObjectType(ECC_GameTraceChannel1);
}




void UPathfinder::InitObstacles()
{
	ISM_Obstacles->ClearInstances();
	for (int i = 0; i < NumberOfObstacles; i++)
	{
		FVector randLocation = GetComponentLocation() + FVector(FMath::RandRange(-(NavigationPlaneExtent.X / 2.0f), (NavigationPlaneExtent.X / 2.0f)), FMath::RandRange(-(NavigationPlaneExtent.Y / 2.0f), (NavigationPlaneExtent.Y / 2.0f)), 0.0f);
		FRotator randRotator = FRotator(0.0f, FMath::RandRange(0.0f, 359.0f), 0.0f);
		FVector randScale = FVector(FMath::RandRange(0.2f, 3.0f), FMath::RandRange(0.1f, 0.2f), 0.2f);
		FTransform randTransform = FTransform(randRotator, randLocation, randScale);
		ISM_Obstacles->AddInstance(randTransform, true);
	}
}


void UPathfinder::InitNavGrid()
{
	//Init array
	gridWidth = (NavigationPlaneExtent.Y / gridSize);
	gridLength = (NavigationPlaneExtent.X / gridSize);


	free(allNodes);
	allNodes = (Node**)malloc(sizeof(Node*) * gridWidth);
	for (int i = 0; i < gridWidth; i++)
	{
		allNodes[i] = (Node*)malloc(sizeof(Node) * (gridLength));
	}
	allNodesSize = (int)(gridWidth * gridLength);


	gridStartLocation = GetComponentLocation() - FVector(NavigationPlaneExtent.X / 2, NavigationPlaneExtent.Y / 2, 0.0f);

	//Init Nodes in grid
	for (int i = 0; i < gridWidth; i++)
	{
		for (int j = 0; j < gridLength; j++)
		{
			FVector tempLocation = gridStartLocation + FVector(gridSize * j, gridSize * i, 0.0f);
			bool tempWalkable = CheckNodeWalkable(tempLocation);

			allNodes[j][i] = Node(tempLocation, tempWalkable, i, j);
		}
	}
}


bool UPathfinder::FindPath(FVector start, FVector finish)
{
	path.Empty();

	if (!CheckNodeWalkable(start) || !CheckNodeWalkable(finish))
		return false;

	Node* startNode = NodeFromLocation(start);
	Node* finishNode = NodeFromLocation(finish);
	
	TArray<Node*> open;
	TArray<Node*> closed;

	open.Add(startNode);

	while (open.Num() > 0)
	{
		Node* currentNode = open[0];
		for (int i = 1; i < open.Num(); i++)
		{
			if (open[i]->fCost() < currentNode->fCost() || open[i]->fCost() == currentNode->fCost())
			{
				if (open[i]->hCost < currentNode->hCost)
				{
					currentNode = open[i];
				}
			}
		}

		open.Remove(currentNode);
		closed.Add(currentNode);


		if (currentNode == finishNode)
		{
			path = RetracePath(startNode, finishNode);
			break;
		}

		
		for (Node* neighbour : GetNeighbours(currentNode))
		{
			if (!neighbour->walkable || closed.Contains(neighbour))
			{
				continue;
			}

			int newCostToNeighbour = currentNode->gCost + GetDistance(currentNode, neighbour);
			if (newCostToNeighbour < neighbour->gCost || !open.Contains(neighbour))
			{
				neighbour->gCost = newCostToNeighbour;
				neighbour->hCost = GetDistance(neighbour, finishNode);
				neighbour->parent = currentNode;

				if (!open.Contains(neighbour))
				{
					open.Add(neighbour);
				}
			}
		}
	}


	if(path.Num() > 0)
	{
		int* i = new int(0);
		(*i) = 0;
		FTimerDelegate TimerDelegate = FTimerDelegate::CreateUObject(this, &UPathfinder::DrawPathSphere, i);
		if(GetWorld()->GetTimerManager().IsTimerActive(AnimationTimerHandle))
			GetWorld()->GetTimerManager().ClearTimer(AnimationTimerHandle);
		GetWorld()->GetTimerManager().SetTimer(AnimationTimerHandle, TimerDelegate, 0.02f, true);

		return true;
	}
	else
	{
		if (GetWorld()->GetTimerManager().IsTimerActive(AnimationTimerHandle))
			GetWorld()->GetTimerManager().ClearTimer(AnimationTimerHandle);
		return false;
	}
}


TArray<UPathfinder::Node*> UPathfinder::RetracePath(Node* startNode, Node* finishNode)
{
	TArray<Node*> newPath;
	Node* currentNode = finishNode;

	while (currentNode != startNode)
	{
		newPath.AddUnique(currentNode);
		currentNode = currentNode->parent;
	}
	Algo::Reverse(newPath);
	return newPath;
}





UPathfinder::Node* UPathfinder::NodeFromLocation(FVector location)
{
	float percentX = FMath::Clamp(((location.X + NavigationPlaneExtent.X / 2) / NavigationPlaneExtent.X), 0.0f, 1.0f);
	float percentY = FMath::Clamp(((location.Y + NavigationPlaneExtent.Y / 2) / NavigationPlaneExtent.Y), 0.0f, 1.0f);

	int x = FMath::RoundToInt((gridLength - 1) * percentX);
	int y = FMath::RoundToInt((gridLength - 1) * percentY);
	return &(allNodes[x][y]);
}

bool UPathfinder::CheckNodeWalkable(FVector worldLocation)
{
	TArray<TEnumAsByte<EObjectTypeQuery>> objectTypes;
	objectTypes.Add(UCollisionProfile::Get()->ConvertToObjectType(ECC_GameTraceChannel1));
	UClass* classFilter = nullptr;
	TArray<AActor*> ActorsToIgnore;

	TArray<UPrimitiveComponent*> output;
	UKismetSystemLibrary::BoxOverlapComponents(GetWorld(), worldLocation, FVector(gridSize / 2.0f, gridSize / 2.0f, 1.0f), objectTypes, classFilter, ActorsToIgnore, output);

	return (output.Num() == 0);
}

TArray<UPathfinder::Node*> UPathfinder::GetNeighbours(Node* node)
{
	TArray<Node*> temp;

	for (int x = -1; x <= 1; x++)
	{
		for (int y = -1; y <= 1; y++)
		{
			if (x == 0 && y == 0)
				continue;

			int checkX = node->gridX + x;
			int checkY = node->gridY + y;

			if (checkX >= 0 && checkX < gridLength && checkY >= 0 && checkY < gridWidth)
			{
				temp.Add(&(allNodes[checkY][checkX]));
			}
		}
	}

	return temp;
}

int UPathfinder::GetDistance(Node* a, Node* b)
{
	int dstX = FMath::Abs(a->gridX - b->gridX);
	int dstY = FMath::Abs(a->gridY - b->gridY);

	if (dstX > dstY)
	{
		return 14 * dstY + 10 * (dstX - dstY);
	}
	return 14 * dstX + 10 * (dstY - dstX);

}

void UPathfinder::DrawPathSphere(int* i)
{
	if (path.Num() > 0)
	{
		if ((*i) >= path.Num())
			(*i) = 0;
		DrawDebugSphere(GetWorld(), path[(*i)]->worldLocation, 10, 12, FColor::Green, false, 0.2f, 0, 1);
		(*i) += 1;

	}
}




void UPathfinder::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UPathfinder::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}



