"let trajectoryfiles = [ `"$(((Get-ChildItem ..\SwervyBot\src\main\deploy\pathplanner\*.path).name) -join '","')`"]" > src/scripts/trajectoryfilelist.js
