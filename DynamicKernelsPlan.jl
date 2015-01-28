

workspace()

# create two models, simple and complex

using SimulatedModelling
using CompassWalkerModel
using KneedWalkerModel

simpleWalkingModel = CompassWalkerModel()
complexWalkingModel = KneedWalkerModel()

# make the simple model walk:
defaultSimpleWalkingGait = getDefaultWalkingGait(simpleWalkingModel)


# control the complex model to walk like the simple one

## first get some states:
initialSimpleState = defaultSimpleWalkingGait.initialState
initialComplexState = getAComplexStateCompatibleWithSimpleState(complexWalkingModel, simpleWalkingModel, initialSimpleState)


currentComplexState = initialComplexState

currentSimpleState = getSimpleModelStateFromComplexModel(complexWalkingModel







# do some estimation (or learning, or regression) of the parameters of the simple model to improve the performance of the complex walker
