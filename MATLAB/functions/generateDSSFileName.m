function name = generateDSSFileName(meshSize, nStepsX, options)

name = "DSSpar_data_";

indices = fieldnames(options);
for i = 1:length(indices)
    value = int8(options.(indices{i}));
    switch indices{i}
        case "onlyAboveX"
            name = name + "oAX" + "-" + value + "_";

        case "optimizeGCI"
            name = name + "oGCI" + "-" + value + "_";

        case "rotatedWS"
            name = name + "rWS" + "-" + value + "_";

        case "senseAtElbows"
            name = name + "sAEl" + "-" + value + "_";

        case "senseAtEither"
            name = name + "sAEi" + "-" + value + "_";

        case "optimizeTrayPosition"
            name = name + "oTP" + "-" + value + "_";

    end
end

name = name + meshSize + "_" + nStepsX;