import {createTheme, responsiveFontSizes} from "@mui/material";

export const theme = responsiveFontSizes(createTheme({
    palette: {
        mode: "dark"
    }
}));
