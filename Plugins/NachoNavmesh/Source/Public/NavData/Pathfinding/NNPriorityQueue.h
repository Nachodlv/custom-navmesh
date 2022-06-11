#pragma once

template <typename InElementType>
class FNNPriorityQueue
{
	template <typename InNodeElementType>
	struct FPriorityQueueNode
	{
		InNodeElementType Element;
		float Priority = 0.0f;

		FPriorityQueueNode() {}

		FPriorityQueueNode(InNodeElementType InElement, float InPriority) : Element(InElement), Priority(InPriority) {}

		friend bool operator<(const FPriorityQueueNode<InNodeElementType>& Lhs, const FPriorityQueueNode<InNodeElementType>& Rhs)
		{
			return Lhs.Priority < Rhs.Priority;
		}
	};
	
public:
	InElementType Pop()
	{
		FPriorityQueueNode<InElementType> Node;
		ElementsArray.HeapPop(Node);
		return Node.Element;
	}

	const InElementType& Peek() const
	{
		const FPriorityQueueNode<InElementType>& Node = ElementsArray.HeapTop();
		return Node.Element;
	}

	void Push(InElementType Element, float Priority)
	{
		FPriorityQueueNode<InElementType> Node (Element, Priority);
		ElementsArray.HeapPush(MoveTemp(Node));
	}
	
	bool IsEmpty() const { return ElementsArray.Num() == 0; }
	
private:
	TArray<FPriorityQueueNode<InElementType>> ElementsArray;
};
