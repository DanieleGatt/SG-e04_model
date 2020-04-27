within ;
package SGE04

  model AccelerationTest "Acceleration test"
    extends VeSyMA.Experiments.Templates.VehicleTest(
      redeclare VeSyMA.Drivers.OpenLoop.OpenLoop driver(
        redeclare Modelica.Blocks.Sources.Ramp acceleratorDemand(startTime=2,
            duration=0.5),
        redeclare Modelica.Blocks.Sources.Trapezoid brakeDemand(
          rising=1,
          width=1,
          falling=0,
          period=2,
          nperiod=1),
        gearboxMode(modeTimeTable={VeSyMA.Drivers.Interfaces.GearboxMode(t=0,
              mode=VeSyMA.Types.GearboxMode.Drive)})),
      redeclare CarQuadElectric vehicle(
        initialGear=1,
        v_start=0,
        motion(stateSelect_xyy=StateSelect.default),
        driveMotor_1(includeCoreMass=false),
        driveMotor_2(includeCoreMass=false),
        driveMotor_3(includeCoreMass=false),
        driveMotor_4(includeCoreMass=false)),
      redeclare VeSyMA.Roads.StaticRoads.FlatRoad road(length=1940),
      redeclare VehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere);

    annotation (
      experiment(StopTime=60, __Dymola_NumberOfIntervals=1000),
      __Dymola_experimentSetupOutput,
      Documentation(revisions="<html>
<table cellspacing=\"0\" cellpadding=\"0\" border=\"0\"><tr>
<td><p><img src=\"modelica://VeSyMA/Images/claytex_logo_100x35px.gif\"/>&nbsp;&nbsp;&nbsp;</p></td>
<td><p><br><b>Copyright &copy; 2015-2019, Claytex Services Limited</b></p></td>
</tr>
</table>
</html>",
      info="<html>
<p>This is an acceleration test.</p>
<p>Utilising an open loop driver model, the vehicle will start from rest, apply the brake to hold the vehicle then accelerate at wide open throttle through the gears throughout the duration of the test.</p>
</html>"),
      __Dymola_Commands(file="modelica://VeSyMA/Scripts/Plot_powertrainVariables.mos"
          "Plot powertrain variables"));
  end AccelerationTest;

  model CarDualElectric
    "Template for a dual electric motor car"
    extends VeSyMA.Automotive.Templates.BaseCar(
      final nForwardGears=1,
      controlType=VeSyMA.Types.VehicleControlType.Automatic,
      redeclare replaceable
        VeSyMA.DriverEnvironments.DriverEnvironments.Automatic
        driverEnvironment(nForwardGears=nForwardGears, redLineFromBus=false),
      redeclare replaceable VeSyMA.Drivelines.MultipleInput.OutboardDirectRear
                                                                       driveline
        constrainedby VeSyMA.Drivelines.Interfaces.DualDriveline,
      redeclare Motorsports.Bodies.Examples.FormulaStudent body,
      redeclare Motorsports.Brakes.Examples.FormulaStudent brakes,
      redeclare Motorsports.HalfCar.Front.FormulaStudent frontAxle,
      redeclare Motorsports.HalfCar.Rear.OpenWheel rearAxle,
      redeclare VeSyMA.Subframes.Ideal.Rigid frontSubframe,
      redeclare VeSyMA.Subframes.Ideal.Rigid rearSubframe,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_1,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_2,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_3,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_4);

    replaceable VeSyMA.Mounts.Rigid                         mountsMotor_1(animation=
         animation) constrainedby VeSyMA.Mounts.Interfaces.BaseMountingSystem(
        animation=animation) "Mount for electric motor 1" annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={-70,-40})));
    replaceable VeSyMA.ElectricMotors.TableBased.AC350Nm driveMotor_1(animation=
          animation) constrainedby VeSyMA.ElectricMotors.Interfaces.DCMotor(
        animation=animation, final motorNumber=1) "Electric Motor 1"
      annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
    replaceable VeSyMA.EnergyStorage.Batteries.StaticSOC      energySource
      constrainedby VeSyMA.EnergyStorage.Interfaces.EnergyStorage(animation=
          animation)
      annotation (Placement(transformation(extent={{-140,-20},{-120,0}})));
    replaceable VeSyMA.ElectricMotors.Controllers.DualElectricMotorController
                                                            motorController
      constrainedby VeSyMA.ElectricMotors.Interfaces.Controller
      annotation (Placement(transformation(extent={{90,90},{110,110}})));
    replaceable VeSyMA.Mounts.Rigid                         mountsMotor_2(animation=
         animation) constrainedby VeSyMA.Mounts.Interfaces.BaseMountingSystem(
        animation=animation) "Mount of motor 2" annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=270,
          origin={-70,70})));
    replaceable VeSyMA.ElectricMotors.TableBased.AC350NmHiSpeed
                                                         driveMotor_2(animation=
          animation) constrainedby VeSyMA.ElectricMotors.Interfaces.DCMotor(
        animation=animation, final motorNumber=2) "Electric Motor 2"
      annotation (Placement(transformation(extent={{-80,50},{-60,30}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-118,-62},{-98,-42}})));
  equation
    connect(driveMotor_1.controlBus, controlBus) annotation (Line(
        points={{-80,-16},{-84,-16},{-84,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(energySource.controlBus, controlBus) annotation (Line(
        points={{-140,-16},{-150,-16},{-150,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(energySource.pin_n, driveMotor_1.pin_n) annotation (Line(
        points={{-120,-16},{-100,-16},{-100,6},{-76,6},{-76,0}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(driveMotor_1.pin_p, energySource.pin_p) annotation (Line(
        points={{-64,0},{-64,10},{-106,10},{-106,-4},{-120,-4}},
        color={0,0,255},
        smooth=Smooth.None));

    connect(driveMotor_1.motorMount, mountsMotor_1.frame_b) annotation (
        Line(
        points={{-70,-20},{-70,-30}},
        color={215,215,215},
        thickness=0.5));
    connect(motorController.controlBus, controlBus) annotation (Line(
        points={{100,90},{100,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(transform_1.frame_b, energySource.chassisFrame) annotation (
        Line(
        points={{0,-88},{0,-80},{-130,-80},{-130,-20}},
        color={95,95,95},
        thickness=0.5));
    connect(driveMotor_2.controlBus, controlBus) annotation (Line(
        points={{-80,46},{-84,46},{-84,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(energySource.pin_n, driveMotor_2.pin_n) annotation (Line(
        points={{-120,-16},{-100,-16},{-100,6},{-76,6},{-76,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(driveMotor_2.pin_p, energySource.pin_p) annotation (Line(
        points={{-64,30},{-64,10},{-106,10},{-106,-4},{-120,-4}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(driveMotor_2.motorMount, mountsMotor_2.frame_b) annotation (
        Line(
        points={{-70,50},{-70,60}},
        color={215,215,215},
        thickness=0.5));
    connect(driveMotor_1.shaft_b, driveline.transmissionFlange) annotation (
       Line(
        points={{-60,-10},{-40,-10}},
        color={135,135,135},
        thickness=0.5));
    connect(driveMotor_2.shaft_b, driveline.transmissionFlange_2)
      annotation (Line(
        points={{-60,40},{-12,40},{-12,-10},{-20,-10}},
        color={135,135,135},
        thickness=0.5));
    connect(driveline.drivelineMount, energySource.chassisFrame) annotation (
        Line(
        points={{-30,-20},{-30,-80},{-130,-80},{-130,-20}},
        color={95,95,95},
        thickness=0.5));
    connect(ground.p, energySource.pin_p) annotation (Line(points={{-108,-42},{
            -106,-42},{-106,-4},{-120,-4}}, color={0,0,255}));
    annotation (
      __Claytex(templateOK=true),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-220,-110},{220,110}},
          initialScale=0.1)),
      Icon(coordinateSystem(
          extent={{-220,-110},{220,110}},
          preserveAspectRatio=false,
          initialScale=0.1)),
      Documentation(revisions="<html>
<table cellspacing=\"0\" cellpadding=\"0\" border=\"0\"><tr>
<td><p><img src=\"modelica://VeSyMA/Images/claytex_logo_100x35px.gif\"/>&nbsp;&nbsp;&nbsp;</p></td>
<td><p><br><b>Copyright &copy; 2015-2019, Claytex Services Limited</b></p></td>
</tr>
</table>
</html>",
      info="<html>
<p>Template of a purely electric vehicle driven with a dual electrical machines.</p>
<p>Varying colours of connections indicate the separation of the systems:</p>
<ul>
<li>Blue Connections - Front hub connections</li>
<li>Green Connections - Rear hub connections</li>
<li>Black Connections - Mounted system connections supported by the <b>mounts</b> block</li>
<li>Light Grey Connections - Mounted system connections supported by the <b>mountMotor_1/2 </b>blocks</li>
</ul>
<h4>Using this Template</h4>
<p>When extending this template: </p>
<ul>
<li>Link the <b>animation</b> parameter in the subsystems to the propagated <b>animation</b> parameter. </li>
<li>Link the <b>mounts </b>block to the relevant position on the subframe system. </li>
<li>Connect <b>driveline.Frame_a</b> to relevant location of chassis frame, before or after the <b>frontSubframe, rearSubframe</b> or <b>mounts</b> blocks. </li>
<li>Ensure the electrical circuit for the electric motor includes a <a href=\"modelica://Modelica.Electrical.Analog.Basic.Ground\">ground</a> block, or similar equivalent. </li>
<li>Link the initialisation parameters in the subsystems to the propagated initialisation parameters. </li>
<li>Connect <b>toHitch.frame_a</b> to relevant location of frame in the chassis systems. <b>toHitch</b>, <b>hitchTransformation</b> and <b>hitchFrame</b> are enabled by the <b>towingHitch</b> Boolean to allow trailers to be connecter to the vehicle, which is deactivated by default. </li>
<li>Finalise the <b>controlType</b> type parameter to match the controls required from the driver model.</li>
<li>Provide a definition for <b>driveline </b>initialisation parameter<b> w_start</b>, this is usually related to the intial driven wheel speed multiplied by the initial drivelline ratio.</li>
</ul>
<p>Note that the parameter <b>nForwardGears </b>has been finalised to 1 as this vehicle template does not contain a multiple gear transmission to reference. </p>
</html>"));
  end CarDualElectric;

  model DualElectricMotorController
    "Controller for a dual motor electric vehicle"
    extends VeSyMA.ElectricMotors.Interfaces.Controller;
  public
    VeSyMA.ElectricMotors.Controllers.Components.RegenerativeBraking regerativeBrakingLogic
      "Logic for regenerative braking demand"
      annotation (Placement(transformation(extent={{-60,-40},{-40,-20}})));
    Modelica.Blocks.Math.Gain gainRegenTorqueDemand_1(k=-43)
      "Regen torque demand for motor 1"
      annotation (Placement(transformation(extent={{10,14},{26,30}})));
    Modelica.Blocks.Math.Add addMotorDemand_1 "Motor demand for motor 1"
      annotation (Placement(transformation(extent={{40,30},{52,42}})));
    Modelica.Blocks.Math.Gain gainRegenTorqueDemand_2(k=-43)
      "Regen torque demand for motor 2"
      annotation (Placement(transformation(extent={{10,-12},{26,4}})));
    Modelica.Blocks.Math.Add addMotorDemand_2 "Motor demand for motor 2"
      annotation (Placement(transformation(extent={{42,-6},{54,6}})));

    Modelica.Blocks.Math.Add addBrakeDemand
      "Adds regen brake demand and park brake demand"
      annotation (Placement(transformation(extent={{74,34},{86,46}})));
    Modelica.Blocks.Math.Gain driveMotorTorqueDemand(k=750)
      "Drive torque demand gain"
      annotation (Placement(transformation(extent={{-58,32},{-42,48}})));
    VeSyMA.Blocks.InputDescriptionsAndUnits.GearboxMode gearboxMode
      "Routing for gearbox mode from the driverBus"
      annotation (Placement(transformation(extent={{-35,77},{-25,87}})));
    Modelica.Blocks.Sources.Constant zeroTorqueDemand(final k=0)
      "Zero torque demand if not in drive or reverse"
      annotation (Placement(transformation(extent={{-58,2},{-42,18}})));
    Claytex.Blocks.Routing.Extractor torqueDemandGearboxMode(final nin=7)
      "Torque demand routing dependent on gearbox mode"
      annotation (Placement(transformation(extent={{-20,50},{0,30}})));
    Modelica.Blocks.Math.Gain reverseMotorTorqueDemand(k=-400)
      "Reverse torque demand gain"
      annotation (Placement(transformation(extent={{-58,62},{-42,78}})));
    Modelica.Blocks.Sources.IntegerConstant parkGearboxMode(final k=integer(
           VeSyMA.Types.GearboxMode.Park)) "Source for park gearbox mode"
      annotation (Placement(transformation(extent={{0,74},{12,86}})));
    Claytex.Blocks.Logical.Integers.IsEqual inPark "=true if in park"
      annotation (Placement(transformation(extent={{24,64},{36,76}})));
    Modelica.Blocks.Logical.TriggeredTrapezoid brakeDemandInPark(
      amplitude=0.5,
      rising=1,
      falling=0) "Source of brake demand when park is selected"
      annotation (Placement(transformation(extent={{44,64},{56,76}})));
    Modelica.Blocks.Math.Add addMotorDemand_3 "Motor demand for motor 2"
      annotation (Placement(transformation(extent={{42,-54},{54,-42}})));
    Modelica.Blocks.Math.Add addMotorDemand_4 "Motor demand for motor 2"
      annotation (Placement(transformation(extent={{42,-28},{54,-16}})));
    Modelica.Blocks.Math.Gain gainRegenTorqueDemand_3(k=-43)
      "Regen torque demand for motor 2"
      annotation (Placement(transformation(extent={{12,-70},{28,-54}})));
    Modelica.Blocks.Math.Gain gainRegenTorqueDemand_4(k=-43)
      "Regen torque demand for motor 2"
      annotation (Placement(transformation(extent={{10,-42},{26,-26}})));
  protected
    VehicleInterfaces.Interfaces.ElectricMotorControlBus
      electricMotorControlBus_2 "Electric motor control bus" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={100,-80})));

    VehicleInterfaces.Interfaces.ElectricMotorBus electricMotorBus_2
      "Electric motor bus" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-100,-80})));
    VehicleInterfaces.Interfaces.ElectricMotorBus electricMotorBus_3
      "Electric motor bus" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-80,-78})));
    VehicleInterfaces.Interfaces.ElectricMotorBus electricMotorBus_4
      "Electric motor bus" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-60,-80})));
  protected
    VehicleInterfaces.Interfaces.ElectricMotorControlBus
      electricMotorControlBus_3 "Electric motor control bus" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={40,-80})));
  protected
    VehicleInterfaces.Interfaces.ElectricMotorControlBus
      electricMotorControlBus_4 "Electric motor control bus" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={70,-80})));
  equation
    connect(regerativeBrakingLogic.brakePedalPositionIn, driverBus.brakePedalPosition)
      annotation (Line(points={{-60.6,-24},{-74,-24},{-74,78},{-100,78},{-100,
            80}}, color={0,0,127}));
    connect(regerativeBrakingLogic.SOC, batteryBus_1.SOC) annotation (Line(
          points={{-60.6,-36},{-86,-36},{-86,0},{-100,0}}, color={0,0,127}));
    connect(regerativeBrakingLogic.vehicleSpeed, chassisBus.longitudinalVelocity)
      annotation (Line(points={{-60.6,-30},{-80,-30},{-80,40},{-100,40}},
          color={0,0,127}));


    connect(gainRegenTorqueDemand_1.u, regerativeBrakingLogic.negativeTorque)
      annotation (Line(points={{8.4,22},{6,22},{6,-35},{-39.2,-35}},
          color={0,0,127}));
    connect(gainRegenTorqueDemand_2.u, regerativeBrakingLogic.negativeTorque)
      annotation (Line(points={{8.4,-4},{6,-4},{6,-35},{-39.2,-35}},
          color={0,0,127}));
    connect(gainRegenTorqueDemand_4.u, regerativeBrakingLogic.negativeTorque)
      annotation (Line(points={{8.4,-34},{6,-34},{6,-35},{-39.2,-35}},  color={0,0,
            127}));
    connect(gainRegenTorqueDemand_3.u, regerativeBrakingLogic.negativeTorque)
      annotation (Line(points={{10.4,-62},{6,-62},{6,-35},{-39.2,-35}}, color={0,0,
            127}));


    connect(addMotorDemand_1.u2, gainRegenTorqueDemand_1.y) annotation (
        Line(points={{38.8,32.4},{40,32.4},{40,22},{26.8,22}},     color={0,
            0,127}));
    connect(addMotorDemand_2.u2, gainRegenTorqueDemand_2.y) annotation (
        Line(points={{40.8,-3.6},{40,-3.6},{40,-4},{26.8,-4}},     color={0,
            0,127}));
    connect(addMotorDemand_4.u2, gainRegenTorqueDemand_4.y) annotation (Line(
          points={{40.8,-25.6},{34,-25.6},{34,-34},{26.8,-34}}, color={0,0,127}));
    connect(addMotorDemand_3.u2, gainRegenTorqueDemand_3.y) annotation (Line(
          points={{40.8,-51.6},{34,-51.6},{34,-62},{28.8,-62}}, color={0,0,127}));



    connect(electricMotorControlBus_2, controlBus.electricMotorControlBus_2)
      annotation (Line(
        points={{100,-80},{100,-100},{0,-100}},
        color={255,204,51},
        thickness=0.5));
    connect(electricMotorControlBus_4, controlBus.electricMotorControlBus_3)
      annotation (Line(
        points={{70,-80},{70,-100},{0,-100}},
        color={255,204,51},
        thickness=0.5));
    connect(electricMotorControlBus_3, controlBus.electricMotorControlBus_4)
      annotation (Line(
        points={{40,-80},{40,-100},{0,-100}},
        color={255,204,51},
        thickness=0.5));


    connect(addMotorDemand_1.y, electricMotorControlBus_1.torqueDemand)
      annotation (Line(points={{52.6,36},{80,36},{80,0},{100,0}},   color={0,
            0,127}));
    connect(addMotorDemand_2.y, electricMotorControlBus_2.torqueDemand)
      annotation (Line(points={{54.6,0},{80,0},{80,-80},{100,-80}},
          color={0,0,127}));
    connect(addMotorDemand_3.y, electricMotorControlBus_3.torqueDemand)
     annotation (Line(
          points={{54.6,-48},{58,-48},{58,-80},{40,-80}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
    connect(addMotorDemand_4.y, electricMotorControlBus_4.torqueDemand)
     annotation (Line(
          points={{54.6,-22},{70,-22},{70,-80}}, color={0,0,127}), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));


    connect(electricMotorBus_2, controlBus.electricMotorBus_2) annotation (
        Line(
        points={{-100,-80},{-100,-100},{0,-100}},
        color={255,204,51},
        thickness=0.5));
    connect(electricMotorBus_3, controlBus.electricMotorBus_3) annotation (Line(
        points={{-80,-78},{-80,-100},{0,-100}},
        color={255,204,51},
        thickness=0.5));
    connect(electricMotorBus_4, controlBus.electricMotorBus_4) annotation (Line(
        points={{-60,-80},{-60,-100},{0,-100}},
        color={255,204,51},
        thickness=0.5));



    connect(addBrakeDemand.y, brakesControlBus.HybridBrakeDemand)
      annotation (Line(points={{86.6,40},{100,40}}, color={0,0,127}));
    connect(driveMotorTorqueDemand.u, driverBus.acceleratorPedalPosition)
      annotation (Line(points={{-59.6,40},{-68,40},{-68,80},{-100,80}},
          color={0,0,127}));
    connect(gearboxMode.gearboxModeInput, driverBus.gearboxMode)
      annotation (Line(points={{-35,82},{-100,82},{-100,80}}, color={255,127,
            0}));
    connect(reverseMotorTorqueDemand.u, driverBus.acceleratorPedalPosition)
      annotation (Line(points={{-59.6,70},{-68,70},{-68,80},{-100,80}},
          color={0,0,127}));
    connect(reverseMotorTorqueDemand.y, torqueDemandGearboxMode.u[2])
      annotation (Line(points={{-41.2,70},{-34,70},{-34,41.1429},{-22,41.1429}},
          color={0,0,127}));
    connect(driveMotorTorqueDemand.y, torqueDemandGearboxMode.u[4])
      annotation (Line(points={{-41.2,40},{-22,40}}, color={0,0,127}));
    connect(zeroTorqueDemand.y, torqueDemandGearboxMode.u[1]) annotation (
        Line(points={{-41.2,10},{-34,10},{-34,38},{-22,38},{-22,41.7143}},
          color={0,0,127}));
    connect(zeroTorqueDemand.y, torqueDemandGearboxMode.u[3]) annotation (
        Line(points={{-41.2,10},{-34,10},{-34,38},{-22,38},{-22,40.5714}},
          color={0,0,127}));
    connect(zeroTorqueDemand.y, torqueDemandGearboxMode.u[5]) annotation (
        Line(points={{-41.2,10},{-34,10},{-34,38},{-22,38},{-22,39.4286}},
          color={0,0,127}));
    connect(zeroTorqueDemand.y, torqueDemandGearboxMode.u[6]) annotation (
        Line(points={{-41.2,10},{-34,10},{-34,38},{-22,38},{-22,38.8571}},
          color={0,0,127}));
    connect(zeroTorqueDemand.y, torqueDemandGearboxMode.u[7]) annotation (
        Line(points={{-41.2,10},{-34,10},{-34,38},{-22,38},{-22,38.2857}},
          color={0,0,127}));
    connect(torqueDemandGearboxMode.index, gearboxMode.gearboxModeOutput)
      annotation (Line(points={{-10,52},{-10,82},{-25,82}}, color={255,127,0}));
    connect(inPark.u2, gearboxMode.gearboxModeOutput) annotation (Line(
          points={{22.8,66.4},{-10,66.4},{-10,82},{-25,82}}, color={255,127,
            0}));
    connect(parkGearboxMode.y, inPark.u1) annotation (Line(points={{12.6,80},
            {16,80},{16,73.6},{22.8,73.6}}, color={255,127,0}));
    connect(brakeDemandInPark.u, inPark.y)
      annotation (Line(points={{42.8,70},{36.6,70}}, color={255,0,255}));
    connect(addBrakeDemand.u1, brakeDemandInPark.y) annotation (Line(points=
           {{72.8,43.6},{60,43.6},{60,70},{56.6,70}}, color={0,0,127}));


    connect(torqueDemandGearboxMode.y, addMotorDemand_1.u1) annotation (
        Line(points={{1,40},{30,40},{30,39.6},{38.8,39.6}},   color={0,0,127}));
    connect(torqueDemandGearboxMode.y, addMotorDemand_2.u1) annotation (
        Line(points={{1,40},{30,40},{30,3.6},{40.8,3.6}},     color={0,0,127}));
    connect(torqueDemandGearboxMode.y, addMotorDemand_4.u1) annotation (Line(
          points={{1,40},{30,40},{30,-18.4},{40.8,-18.4}}, color={0,0,127}));
    connect(torqueDemandGearboxMode.y, addMotorDemand_3.u1) annotation (Line(
          points={{1,40},{30,40},{30,-44.4},{40.8,-44.4}}, color={0,0,127}));


    connect(regerativeBrakingLogic.brakeDemandOut, addBrakeDemand.u2)
      annotation (Line(points={{-39.2,-25},{2,-25},{2,50},{82,50},{82,36.4},{72.8,
            36.4}},       color={0,0,127}));


    annotation (Documentation(info="<html>
<p>This controller is intended for a dual motor electric vehicle, it multiplies the demand from the driver accelerator to gain a torque demand from the motor and then uses the <a href=\"modelica://VeSyMA.ElectricMotors.Controllers.RegenerativeOnBrakes\">regenerativeBraking</a> logic to control the demand for negative torque from both motors and additional demand for the brakes; The brakes include a modifier within <a href=\"modelica://VeSyMA.Brakes.IndependentBrakes.Real1DBrakesHybrid\">Real1DHybrid</a> model which add brake demand onto the demand from the driver environment. The brake demand from the driver environment can be modified to include a dead zone by using a lookup table based output such as <a href=\"modelica://Claytex.Blocks.Conversions.FromNormalised.TranslationalForceLookup\">TranslationalForceLookup</a> and <a href=\"modelica://Claytex.Blocks.Conversions.FromNormalised.TranslationalPositionLookup\">TranslationalPositionLookup</a>.</p>
<p>The gearbox mode input is used to control the torque demand out to the electric motor. If the gearboxMode = <a href=\"modelica://VeSyMA.Types.GearboxMode\">Park</a> then the brakes are ramped up and then released when moved out of park; if in <a href=\"modelica://VeSyMA.Types.GearboxMode\">Reverse</a> then the <b>reverseMotorTorqueDemand</b> gain is used; if in <a href=\"modelica://VeSyMA.Types.GearboxMode\">Drive</a> then the <b>driveMotorTorqueDemand</b> gain, else zero torque output is demanded.</p>
</html>",
      revisions="<html>
<table cellspacing=\"0\" cellpadding=\"0\" border=\"0\"><tr>
<td><p><img src=\"modelica://VeSyMA/Images/claytex_logo_100x35px.gif\"/>&nbsp;&nbsp;&nbsp;</p></td>
<td><p><br><b>Copyright &copy; 2015-2019, Claytex Services Limited</b></p></td>
</tr>
</table>
</html>"));
  end DualElectricMotorController;

  model CarQuadElectric
    "Template for a quad electric motor car"
    extends VeSyMA.Automotive.Templates.BaseCar(
      final nForwardGears=1,
      controlType=VeSyMA.Types.VehicleControlType.Automatic,
      redeclare replaceable
        VeSyMA.DriverEnvironments.DriverEnvironments.Automatic
        driverEnvironment(nForwardGears=nForwardGears, redLineFromBus=false),
      redeclare replaceable VeSyMA.Drivelines.MultipleInput.QuadOutboardDirect
                                                                       driveline
        constrainedby VeSyMA.Drivelines.Interfaces.QuadDriveline,
      redeclare Motorsports.Brakes.Examples.FormulaStudent brakes,
      redeclare Motorsports.HalfCar.Front.FormulaStudent frontAxle,
      redeclare Motorsports.HalfCar.Rear.RoadCar rearAxle,
      redeclare Motorsports.Bodies.Examples.FormulaStudent body,
      redeclare VeSyMA.Subframes.Ideal.Rigid frontSubframe,
      redeclare VeSyMA.Subframes.Ideal.Rigid rearSubframe,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_1,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_2,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_3,
      redeclare Motorsports.WheelsAndTyres.Examples.FormulaStudent wheel_4);

    replaceable VeSyMA.Mounts.Rigid                         mountsMotor_1(animation=
         animation) constrainedby VeSyMA.Mounts.Interfaces.BaseMountingSystem(
        animation=animation) "Mount for electric motor 1" annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=0,
          origin={-70,-40})));
    replaceable VeSyMA.ElectricMotors.TableBased.AC350NmHiSpeed
                                                         driveMotor_1(animation=
          animation) constrainedby VeSyMA.ElectricMotors.Interfaces.DCMotor(
        animation=animation, final motorNumber=1) "Electric motor 1"
      annotation (Placement(transformation(extent={{-120,-30},{-100,-10}})));
    replaceable VeSyMA.EnergyStorage.Batteries.StaticSOC      energySource
      constrainedby VeSyMA.EnergyStorage.Interfaces.EnergyStorage(animation=
          animation)
      annotation (Placement(transformation(extent={{-210,-10},{-190,10}})));
    replaceable DualElectricMotorController                 motorController
      constrainedby VeSyMA.ElectricMotors.Interfaces.Controller
      annotation (Placement(transformation(extent={{90,90},{110,110}})));
    replaceable VeSyMA.Mounts.Rigid                         mountsMotor_2(animation=
         animation) constrainedby VeSyMA.Mounts.Interfaces.BaseMountingSystem(
        animation=animation) "Mount of motor 2" annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=270,
          origin={-70,40})));
    replaceable VeSyMA.ElectricMotors.TableBased.AC350NmHiSpeed
                                                         driveMotor_2(animation=
          animation) constrainedby VeSyMA.ElectricMotors.Interfaces.DCMotor(
        animation=animation, final motorNumber=2) "Electric motor 2"
      annotation (Placement(transformation(extent={{-120,50},{-100,30}})));
    replaceable VeSyMA.Mounts.Rigid                         mountsMotor_3(animation=
         animation) constrainedby VeSyMA.Mounts.Interfaces.BaseMountingSystem(
        animation=animation) "Mount for electric motor 3" annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={-160,-30})));
    replaceable VeSyMA.ElectricMotors.TableBased.AC350NmHiSpeed
                                                         driveMotor_3(animation=
          animation) constrainedby VeSyMA.ElectricMotors.Interfaces.DCMotor(
        animation=animation, final motorNumber=3) "Electric motor 3"
      annotation (Placement(transformation(extent={{-170,-50},{-150,-70}})));
    replaceable VeSyMA.Mounts.Rigid                         mountsMotor_4(animation=
         animation) constrainedby VeSyMA.Mounts.Interfaces.BaseMountingSystem(
        animation=animation) "Mount for electric motor 4" annotation (Placement(
          transformation(
          extent={{10,10},{-10,-10}},
          rotation=270,
          origin={-160,40})));
    replaceable VeSyMA.ElectricMotors.TableBased.AC350NmHiSpeed
                                                         driveMotor_4(animation=
          animation) constrainedby VeSyMA.ElectricMotors.Interfaces.DCMotor(
        animation=animation, final motorNumber=4) "Electric motor 4"
      annotation (Placement(transformation(extent={{-170,60},{-150,80}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{-194,-110},{-174,-90}})));
  equation
    connect(driveMotor_1.controlBus, controlBus) annotation (Line(
        points={{-120,-26},{-124,-26},{-124,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(energySource.controlBus, controlBus) annotation (Line(
        points={{-210,-6},{-220,-6},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(energySource.pin_n, driveMotor_1.pin_n) annotation (Line(
        points={{-190,-6},{-116,-6},{-116,-10}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(driveMotor_1.pin_p, energySource.pin_p) annotation (Line(
        points={{-104,-10},{-104,6},{-190,6}},
        color={0,0,255},
        smooth=Smooth.None));

    connect(driveMotor_1.motorMount, mountsMotor_1.frame_b) annotation (
        Line(
        points={{-110,-30},{-110,-40},{-80,-40}},
        color={215,215,215},
        thickness=0.5));
    connect(motorController.controlBus, controlBus) annotation (Line(
        points={{100,90},{100,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(driveMotor_2.controlBus, controlBus) annotation (Line(
        points={{-120,46},{-124,46},{-124,22},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(energySource.pin_n, driveMotor_2.pin_n) annotation (Line(
        points={{-190,-6},{-116,-6},{-116,30}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(driveMotor_2.pin_p, energySource.pin_p) annotation (Line(
        points={{-104,30},{-104,6},{-190,6}},
        color={0,0,255},
        smooth=Smooth.None));
    connect(driveMotor_2.motorMount, mountsMotor_2.frame_b) annotation (
        Line(
        points={{-110,50},{-110,60},{-70,60},{-70,50}},
        color={215,215,215},
        thickness=0.5));
    connect(driveMotor_3.motorMount, mountsMotor_3.frame_b) annotation (
        Line(
        points={{-160,-50},{-160,-40}},
        color={215,215,215},
        thickness=0.5));
    connect(driveMotor_3.controlBus, controlBus) annotation (Line(
        points={{-170,-54},{-220,-54},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(driveMotor_4.controlBus, controlBus) annotation (Line(
        points={{-170,64},{-220,64},{-220,22}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(driveMotor_4.motorMount, mountsMotor_4.frame_b) annotation (
        Line(
        points={{-160,60},{-160,50}},
        color={215,215,215},
        thickness=0.5));
    connect(driveMotor_4.pin_n, energySource.pin_n) annotation (Line(points=
           {{-166,80},{-166,84},{-180,84},{-180,-6},{-190,-6}}, color={0,0,255}));
    connect(driveMotor_4.pin_p, energySource.pin_p) annotation (Line(points=
           {{-154,80},{-154,88},{-184,88},{-184,6},{-190,6}}, color={0,0,255}));
    connect(driveMotor_3.pin_p, energySource.pin_p) annotation (Line(points=
           {{-154,-70},{-154,-78},{-184,-78},{-184,6},{-190,6}}, color={0,0,
            255}));
    connect(driveMotor_1.shaft_b, driveline.transmissionFlange_1)
      annotation (Line(
        points={{-100,-20},{-92,-20},{-92,-16},{-40,-16}},
        color={135,135,135},
        thickness=0.5));
    connect(driveMotor_2.shaft_b, driveline.transmissionFlange) annotation (
       Line(
        points={{-100,40},{-92,40},{-92,-10},{-40,-10}},
        color={135,135,135},
        thickness=0.5));
    connect(driveMotor_3.pin_n, energySource.pin_n) annotation (Line(points=
           {{-166,-70},{-166,-74},{-180,-74},{-180,-6},{-190,-6}}, color={0,
            0,255}));
    connect(energySource.chassisFrame, transform_1.frame_b) annotation (
        Line(
        points={{-200,-10},{-200,-80},{4.44089e-016,-80},{4.44089e-016,-88}},
        color={95,95,95},
        thickness=0.5));

    connect(driveMotor_4.shaft_b, driveline.transmissionFlange_4)
      annotation (Line(
        points={{-150,70},{-40,70},{-40,40},{-12,40},{-12,-10},{-20,-10}},
        color={135,135,135},
        thickness=0.5));
    connect(driveMotor_3.shaft_b, driveline.transmissionFlange_3)
      annotation (Line(
        points={{-150,-60},{-12,-60},{-12,-16},{-20,-16}},
        color={135,135,135},
        thickness=0.5));
    connect(driveline.drivelineMount, transform_1.frame_b) annotation (Line(
        points={{-30,-20},{-30,-80},{4.44089e-16,-80},{4.44089e-16,-88},{
            3.33067e-16,-88}},
        color={95,95,95},
        thickness=0.5));
    connect(ground.p, energySource.pin_p) annotation (Line(points={{-184,-90},{
            -184,6},{-190,6}}, color={0,0,255}));
    annotation (
      __Claytex(templateOK=true),
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-220,-110},{220,110}},
          initialScale=0.1)),
      Icon(coordinateSystem(
          extent={{-220,-110},{220,110}},
          preserveAspectRatio=false,
          initialScale=0.1)),
      Documentation(revisions="<html>
<table cellspacing=\"0\" cellpadding=\"0\" border=\"0\"><tr>
<td><p><img src=\"modelica://VeSyMA/Images/claytex_logo_100x35px.gif\"/>&nbsp;&nbsp;&nbsp;</p></td>
<td><p><br><b>Copyright &copy; 2015-2019, Claytex Services Limited</b></p></td>
</tr>
</table>
</html>",
      info="<html>
<p>Template of a purely electric vehicle driven with a four electrical machines.</p>
<p>Varying colours of connections indicate the separation of the systems:</p>
<ul>
<li>Blue Connections - Front hub connections</li>
<li>Green Connections - Rear hub connections</li>
<li>Black Connections - Mounted system connections supported by the <b>mounts</b> block</li>
<li>Light Grey Connections - Mounted system connections supported by the <b>mountMotor_1/2/3/4</b> blocks</li>
</ul>
<h4>Using this Template</h4>
<p>When extending this template: </p>
<ul>
<li>Link the <b>animation</b> parameter in the subsystems to the propagated <b>animation</b> parameter. </li>
<li>Link the <b>mounts </b>block to the relevant position on the subframe system. </li>
<li>Connect <b>driveline.Frame_a</b> to relevant location of chassis frame, before or after the <b>frontSubframe, rearSubframe</b> or <b>mounts</b> blocks. </li>
<li>Ensure the electrical circuit for the electric motor includes a <a href=\"modelica://Modelica.Electrical.Analog.Basic.Ground\">ground</a> block, or similar equivalent. </li>
<li>Link the initialisation parameters in the subsystems to the propagated initialisation parameters. </li>
<li>Connect <b>toHitch.frame_a</b> to relevant location of frame in the chassis systems. <b>toHitch</b>, <b>hitchTransformation</b> and<b> hitchFrame</b> are enabled by the <b>towingHitch</b> Boolean to allow trailers to be connecter to the vehicle, which is deactivated by default. </li>
<li>Finalise the <b>controlType</b> type parameter to match the controls required from the driver model.</li>
<li>Provide a definition for <b>driveline </b>initialisation parameter <b>w_start</b>, this is usually related to the intial driven wheel speed multiplied by the initial drivelline ratio.</li>
</ul>
<p>Note that the parameter<b> nForwardGears </b>has been finalised to 1 as this vehicle template does not contain a multiple gear transmission to reference. </p>
</html>"));
  end CarQuadElectric;
  annotation (uses(
      VeSyMA(version="2019.2"),
      Modelica(version="3.2.3"),
      VehicleInterfaces(version="1.2.5"),
      Motorsports(version="2019.2"),
      Claytex(version="2019.2")));
end SGE04;
