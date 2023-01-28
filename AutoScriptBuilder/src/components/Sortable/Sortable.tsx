import React, {useState} from "react";
import {
    SortableContext,
    sortableKeyboardCoordinates,
    useSortable,
    verticalListSortingStrategy
} from "@dnd-kit/sortable";
import {CSS} from "@dnd-kit/utilities";
import {
    DndContext,
    DragOverlay,
    KeyboardSensor,
    MouseSensor,
    PointerSensor,
    pointerWithin,
    TouchSensor,
    UniqueIdentifier,
    useSensor,
    useSensors
} from "@dnd-kit/core";
import styles from './Sortable.module.scss';
import {createPortal} from "react-dom";
import classNames from "classnames";
import { Grid } from "@mui/material";

interface ISortableItemProps {
    children: any;
    index: number;
    item: any;
    handle?: boolean;
}

function SortableItem(props: ISortableItemProps) {
    const {
        attributes,
        listeners,
        isDragging,
        setNodeRef,
        transform,
        transition
    } = useSortable({
        id: props.item.id,
        transition: {
            duration: 150, easing: 'cubic-bezier(0.25, 1, 0.5, 1)'
        },
    });

    const style = {
        transform: CSS.Translate.toString(transform),
        transition,
        touchAction: "none"
    };

    return (
        <div className={classNames(isDragging && styles.dragging)} ref={setNodeRef}
             style={{...style}} {...attributes}
             tabIndex={!props.handle ? 0 : undefined}>
            {props.children({index: props.index, item: props.item, listeners: listeners})}
        </div>
    );
}

interface IProps {
    // Content
    children: any;
    content: Array<any>;

    // Functions
    onDragEnd: any;

    // Properties
    modifiers?: any;
    handle?: boolean;
    dragOverlay?: boolean;
}

/* Simple sorting that uses the built-in dnd-kit sortable to sort a given list of
   items. Provided a child render function for custom rendering */
export const Sortable = ({
                                   children,
                                   content,
                                   onDragEnd,
                                   modifiers,
                                   handle = false,
                                   dragOverlay = false
                               }: IProps) => {
    const [activeId, setActiveId] = useState<UniqueIdentifier | null>(null);
    const sensors = useSensors(
        useSensor(MouseSensor, {
            activationConstraint: {
                distance: 0,
            },
        }),
        useSensor(PointerSensor),
        useSensor(TouchSensor, {
            activationConstraint: {
                delay: 250,
                tolerance: 5,
            },
        }),
        useSensor(KeyboardSensor, {
            coordinateGetter: sortableKeyboardCoordinates,
        })
    );

    const getIndex = (id: UniqueIdentifier) =>
        content.findIndex((item: any) => item.id === id);

    return (
        <Grid container spacing={3} style={{width: "100%"}}>
            <DndContext
                onDragStart={({active}) => {
                    if (!active)
                        return;
                    setActiveId(active.id);
                }}
                onDragEnd={({over}) => {
                    if (over) {
                        if (over && activeId) {
                            const activeIndex = getIndex(activeId);
                            const overIndex = getIndex(over.id);
                            if (activeIndex !== overIndex) {
                                onDragEnd({oldIndex: activeIndex, newIndex: overIndex});
                            }
                        }
                    }
                }}
                sensors={sensors}
                modifiers={modifiers}
                collisionDetection={pointerWithin}
            >
                <SortableContext items={content.map((item: any) => item.id)}
                                 strategy={verticalListSortingStrategy} data-no-dnd={"true"}>
                    <div style={{display: "flex", flexDirection: "column", width: "100%"}}>
                        {content.map((item: any, index: number) => (
                            <SortableItem key={`${index}-${item.id}`} index={index} item={item} children={children}
                                          handle={handle}/>
                        ))}
                    </div>
                </SortableContext>

                {dragOverlay && (
                    <>{createPortal(
                        <DragOverlay>
                            {activeId ?
                                <>
                                    {children(
                                        {
                                            item: content.find((item) => item.id === activeId),
                                            listeners: null
                                        }
                                    )}
                                </>
                                : null}
                        </DragOverlay>, document.body
                    )}</>
                )}
            </DndContext>
        </Grid>
    );
}


