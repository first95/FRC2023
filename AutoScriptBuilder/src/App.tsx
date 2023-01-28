import {Divider, styled as smc, Typography} from "@mui/material";
import styled from "styled-components";
import {Sortable} from "./components/Sortable/Sortable";
import {restrictToHorizontalAxis, restrictToWindowEdges} from "@dnd-kit/modifiers";
import {useState} from "react";
import {arrayMove} from "@dnd-kit/sortable";
import {DragIndicator} from "@mui/icons-material";
import Command from "./components/Command";

export default function App() {
    const [commands, setCommands] = useState([]);
    const [builder, setBuilder] = useState([]);

    const onDragEnd = ({oldIndex, newIndex}: any) => {
        setBuilder(arrayMove(builder, oldIndex, newIndex));
    };

    return (
        <Container>
            <Title>
                FRC 95 Auton Builder
            </Title>
            <Divider />

            <Subtitle>Command Bank</Subtitle>
            <Content>
                <Command id={"test"} title={"test"}/>
            </Content>

            <Subtitle>Builder</Subtitle>
            <Content>
                <Sortable
                    content={builder}
                    onDragEnd={onDragEnd}
                    modifiers={[restrictToHorizontalAxis, restrictToWindowEdges]}
                    handle={true}
                >
                    {(renderProps: any) => {
                        return (
                            <div style={{
                                flex: 1,
                                display: "flex",
                                alignItems: "center"
                            }}>
                                <div style={{cursor: "grab"}} {...renderProps.listeners} draggable>
                                    <DragIndicator/>
                                </div>
                                <h6 style={{textAlign: 'left'}}>TEST</h6>
                            </div>
                        )
                    }}
                </Sortable>

            </Content>
        </Container>
    )
}

const Container = styled.div`
  display: flex;
  flex-direction: column;
`;

const Content = styled.div`
  margin: 1em;
  width: calc(100% - 2em);
  min-height: 250px;

  border-radius: 20px;
  background-color: rgba(217, 217, 217, 0.1);
`;

const Title = smc(Typography)`
  margin: 1em;
  font-size: 32px;
  text-align: left;
`;

const Subtitle = smc(Typography)`
    margin: 1em 1em 0 1em;
    font-size: 24px;
`;
