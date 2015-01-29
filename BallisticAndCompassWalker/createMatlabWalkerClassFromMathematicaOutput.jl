


walkerName = "CompassWalker"

propertiesList = ["L", "MLeg", "ILeg", "MPelvis", "IPelvis"]
qAndUIndeces = {1:6, 7:12}
states = ["stanceFootHinged"]


superClass = "Walker"



##

mathematicaOutputFilename = "mathematicaOutput$(walkerName).m"
walkerClassFilename = "$(walkerName)ExportedClass.m"
stateName = "$(walkerName)State"

cd("C:\\Users\\john\\workspace\\DynamicsKernels\\BallisticAndCompassWalker")


## this section looks through the code that mathematica outputted and breaks it into chunks

file = open(mathematicaOutputFilename)
allLines = readlines(file)
close(file)

function lookAtLineForTagDelimiter(startKey, endKey, indeces, line, lineNumber)
    line = uppercase(line)
    startKey = uppercase(startKey)

    if length(search(line, startKey)) > 0
        indeces[1] = min(lineNumber, indeces[1])
    end

    if !isempty(endKey)
        endKey = uppercase(endKey)
        if length(search(line, endKey)) > 0
            indeces[2] = lineNumber
        end
    end
    return indeces
end

function getLinesFromIndeces(allLines, indeces)
    return allLines[indeces[1]:indeces[2]]
end

function parseMathematicaOutput(allLines, states)
    statesAndTrigLineIndeces = Inf * ones(2);
    massMatrixAndRHSIndeces = Inf * ones(2);
    constraintMatrixIndeces = Inf * ones(length(states), 2);
    energiesIndeces = Inf * ones(2);
    kinematicsIndeces = Inf * ones(2);

    shouldPrintOutAllParsedIndeces = false;

    for lineNumber = 1 : length(allLines)
        line = allLines[lineNumber]
        stateAssignmentSearch = length(search(line, "% State assignments"))
        print(line)

        statesAndTrigLineIndeces = 
            lookAtLineForTagDelimiter("% State assignments", "% Mass Matrix", 
                statesAndTrigLineIndeces, line, lineNumber)

        massMatrixAndRHSIndeces = 
            lookAtLineForTagDelimiter("MM =", "udot = MM\\rhs;",
                massMatrixAndRHSIndeces, line, lineNumber)

        for stateNumber = 1:length(states)
            state = states[stateNumber];
            startKey = "constraintJacobian$(state)(1,1) ="
            endKey = "constraintJacobian$(state)Dot"
            constraintMatrixIndeces[stateNumber, :] = 
                lookAtLineForTagDelimiter(startKey, endKey,
                    constraintMatrixIndeces[stateNumber, :], line, lineNumber)
        end


        energiesIndeces = 
            lookAtLineForTagDelimiter("energies.", [],
                energiesIndeces, line, lineNumber)

        kinematicsIndeces = 
            lookAtLineForTagDelimiter("points.", "points.",
                kinematicsIndeces, line, lineNumber)

    end

    energiesIndeces[2] = kinematicsIndeces[1] - 1
    kinematicsIndeces[2] = length(allLines)

    massMatrixAndRHSIndeces[2] -= 1



    parsedLines = Dict();

    parsedLines[:statesAndTrig] = getLinesFromIndeces(allLines, statesAndTrigLineIndeces);
    parsedLines[:massMatrixAndRHS] = getLinesFromIndeces(allLines, massMatrixAndRHSIndeces);

    constraintMatrices = Dict();
    for stateNumber = 1:length(states)
        state = states[stateNumber];
        constraintMatrices[symbol(state)] = getLinesFromIndeces(allLines, 
            constraintMatrixIndeces[stateNumber, :]);
    end
    parsedLines[:constraintMatrices] = constraintMatrices;

    parsedLines[:energies] = getLinesFromIndeces(allLines, energiesIndeces);
    parsedLines[:kinematics] = getLinesFromIndeces(allLines, kinematicsIndeces);

    if shouldPrintOutAllParsedIndeces
        println("statesAndTrigLineIndeces = $statesAndTrigLineIndeces")
        println("massMatrixAndRHSIndeces = $massMatrixAndRHSIndeces")

        for stateNumber = 1:length(states)
            state = states[stateNumber];
            print("$state, ")
        end
        println(constraintMatrixIndeces)

        println("energiesIndeces = $energiesIndeces")
        println("kinematicsIndeces = $kinematicsIndeces")
    end

    return parsedLines
end

parsedMathematicaOutput = parseMathematicaOutput(allLines, states)




## This part writes out a file that defines a matlab walker type class with the chunks of code from
# the mathematica organized into functions, with a few other helpful functions.


file = open(walkerClassFilename, "w")
out(s) = print(file, s) #print(s) #

out(tabLevel, s) = out(*(["  " for i = 1:tabLevel]...) * s)

writeLinesOut(tab, lines) = begin
    for line in lines
        out(tab, line)
    end
end

writeLinesOut(lines) = begin
    global tab
    writeLinesOut(tab, lines)
end


beginMatlabBlock(name) = begin
    global tab
    out(tab, "$(name)\n")
    tab += 1;
end

endMatlabBlock(s) = begin
    global tab
    tab -= 1;
    #out("\n")
    out(tab, "end\n$(s)")
end
endMatlabBlock() = endMatlabBlock("")

endClassSectionBlock() = begin
    global tab
    out(tab, "\n")
    endMatlabBlock("\n")
end

beginMethodBlock(outputs, functionName, realInputs) = beginMatlabBlock("function [$(outputs)] = $(functionName)(this, $(realInputs))")
beginMethodBlock(outputs, functionName) = beginMatlabBlock("function [$(outputs)] = $(functionName)(this)")




writeConstructor(walkerName, superClass, propertiesList) = begin
    global tab
    beginMatlabBlock("function [this] = $(walkerName)(input)")
    out(tab, "this = this@$(superClass)();\n")
    
    beginMatlabBlock("if (nargin == 1 && ~isempty(input))")
    for p in propertiesList
        out(tab, "this.$(p) = input.$(p)\n")
    end
    endMatlabBlock()

    endMatlabBlock("\n")
end

writeStateGetter(walkerName, stateName) = begin
    global tab
    beginMethodBlock("state", "getWalkerStateObjectFromVector", "stateVector")
    out(tab, "state = $(stateName)(stateVector);\n")
    endMatlabBlock("\n")
end

writeIndecesGetter(qs, us) = begin
    beginMethodBlock("qs, us", "getWalkerStateObjectFromVector")
    global tab
    out(tab, "qs = $(qs);\n")
    out(tab, "us = $(us);\n")
    endMatlabBlock("\n")
end

writeParameterSetterInCurrentFunction(propertiesList) = begin
    beginMethodBlock("", "setWalkerParamsInCurrentFunction")

    global tab
    out(tab, "ws = \'caller\';\n")
    for p in propertiesList
        out(tab, "assignin(ws, '$(p)', this.$(p));\n")
    end

    endMatlabBlock("\n")
end

writeConvertStateToVector(tab) = out(tab, "state = this.getWalkerStateObjectFromVector(state);\n")
writeSetStatesAndParamsInCurrentFunction(tab) = begin     
    out(tab, "state.setQsUsAndTrigInCurrentFunction();\n")
    out(tab, "this.setWalkerParamsInCurrentFunction();\n\n")
end

writeConvertStateToVector() = begin
    global tab
    writeConvertStateToVector(tab)
end
writeSetStatesAndParamsInCurrentFunction() = begin
    global tab
    writeSetStatesAndParamsInCurrentFunction(tab)
end



writeGetKinematicsFunction(kinematicsText) = begin
    beginMethodBlock("points", "getKinematicPoints", "state, uDot")
    writeSetStatesAndParamsInCurrentFunction()
    writeLinesOut(kinematicsText)
    endMatlabBlock("\n")
end

writeGetMassMatrixAndRHS(massMatrixAndRHSText) = begin
    beginMethodBlock("MM, rhs", "getMassMatrixAndRightHandSide", "time, state")
    writeConvertStateToVector()
    writeSetStatesAndParamsInCurrentFunction()

    out(tab, "for i = 1 : length(this.controllers)\n")
    out(tab+1, "this.controllers{i}.calculateControlAndSetInCurrentFunction(this, time, state);\n")
    out(tab, "end\n\n")

    writeLinesOut(massMatrixAndRHSText)
    endMatlabBlock("\n")
end

getLargestMatrixDimensionsReferencedInStrings(strs) = begin
    largestDims = zeros(2)
    for str in strs
        firstMatch = match(r"\(.?\,", str)
        secondMatch = match(r"\,.?\)", str)

        if firstMatch != nothing
            firstMatch = firstMatch.match[2:(end-1)]
            largestDims[1] = max(largestDims[1], parseint(firstMatch))
        end

        if secondMatch != nothing
            secondMatch = secondMatch.match[2:(end-1)]
            largestDims[2] = max(largestDims[2], parseint(secondMatch))
        end
    end
    return largestDims
end


writeGetConstraintMatrices(constraintMatricesLines, states) = begin
    beginMethodBlock("C, CDot", "getConstraintMatrix", "state, mode")
    writeConvertStateToVector()
    writeSetStatesAndParamsInCurrentFunction()

    beginMatlabBlock("switch mode")

    global tab
    for state in states
        constraintMatrixText = constraintMatricesLines[symbol(state)]
        largestDims = getLargestMatrixDimensionsReferencedInStrings(constraintMatrixText)
        dims = Int64[];
        push!(dims, largestDims[1])
        push!(dims, largestDims[2])

        beginMatlabBlock("case \'$(state)\'")

        stateUppered = string(uppercase(state[1])) * state[2:end]
        jacobianNameRoot = "constraintJacobian$(stateUppered)"
        out(tab, "$(jacobianNameRoot)($(dims[1]),$(dims[2])) = 0;\n")
        out(tab, "$(jacobianNameRoot)Dot($(dims[1]),$(dims[2])) = 0;\n\n")

        writeLinesOut(tab, constraintMatrixText)

        out(tab, "\n")
        out(tab, "C = $(jacobianNameRoot);\n")
        out(tab, "CDot = $(jacobianNameRoot)Dot;\n\n")

        tab -= 1;
    end

    beginMatlabBlock("otherwise")
    out(tab, "error(\'unknown mode for walker: %s\', mode);\n")

    endMatlabBlock()
    endMatlabBlock("\n")
end


writeGetEnergyFunction(energyLines) = begin
    beginMethodBlock("energies", "getEnergyOfState", "state")
    writeConvertStateToVector()
    writeSetStatesAndParamsInCurrentFunction()
    writeLinesOut(energyLines)
    endMatlabBlock("\n")
end






global tab = 0;

beginMatlabBlock("classdef $(walkerName) < $(superClass)\n") 

beginMatlabBlock("properties\n")
for p in propertiesList
    out(tab, "$p = 0;\n")
end
out(tab, "\n")
out(tab, "controllers = {};\n")
endClassSectionBlock()


beginMatlabBlock("methods\n")
writeConstructor(walkerName, superClass, propertiesList)
writeStateGetter(walkerName, stateName)
writeIndecesGetter(qAndUIndeces[1], qAndUIndeces[2])
writeParameterSetterInCurrentFunction(propertiesList)
writeGetKinematicsFunction(parsedMathematicaOutput[:kinematics])
writeGetMassMatrixAndRHS(parsedMathematicaOutput[:massMatrixAndRHS])
writeGetConstraintMatrices(parsedMathematicaOutput[:constraintMatrices], states)
writeGetEnergyFunction(parsedMathematicaOutput[:energies])
endMatlabBlock("\n")


endClassSectionBlock()

close(file)




## this part writes out another file, this defines a matlab class representing the state of the 
# walker we just created.


# TODO!



# functions to consider adding to a stub subclass of this walker:
#     function [speed] = calculateSpeed(this, xInitial, xFinal, tFinal)
#     function [stepLength] = calculateStepLength(this, xInitial, xFinal)



