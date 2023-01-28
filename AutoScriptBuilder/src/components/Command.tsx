import {Button, styled as smc, Tooltip, Typography} from "@mui/material";

interface IProps {
    id: string;
    title: string;
}

export default function Command(props: IProps) {
    return (
        <Tooltip title={""}>
            <Container variant="contained" key={props.id}>
                <Typography>{props.title}</Typography>
            </Container>
        </Tooltip>
    );
}

const Container = smc(Button)`
  width: 125px;
  height: 50px;

  margin: 1em;

  display: flex;
  justify-content: center;
  align-items: center;

  box-shadow: 0px 10px 30px -12px rgba(0, 0, 0, 0.65);
  -webkit-box-shadow: 0px 10px 30px -12px rgba(0, 0, 0, 0.65);

  // background: #1b1b1b;
  border-radius: 10px;
`;
