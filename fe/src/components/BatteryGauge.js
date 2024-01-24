import React from "react";

import GaugeBg from "../assets/gauge-bg.png";

const gaugeWidth = 60;
const gaugeHeight = 30;
const gaugeContentWidth = gaugeWidth - 10;
const gaugeBarsNb = 3;
const gaugeBarWidth = gaugeContentWidth / gaugeBarsNb;
const gaugeBarMargin = 2;
const gaugeBarRadius = 4;

const styles = {
  container: {
    position: "relative",
    width: `${gaugeWidth}px`,
    height: `${gaugeHeight}px`,
  },
  barsContainer: {
    width: `${gaugeWidth}px`,
    height: `${gaugeHeight}px`,
    display: "flex",
    flexDirection: "row",
    alignItems: "center",
    marginLeft: "3px",
  },
  barContainer: {
    width: `${gaugeBarWidth}px`,
    height: `${gaugeHeight - 10}px`,
    paddingLeft: `${gaugeBarMargin}px`,
    paddingRight: `${gaugeBarMargin}px`,
  },
  bar: {
    width: `${gaugeBarWidth - gaugeBarMargin * 2}px`,
    height: "100%",
    backgroundColor: "#3f5c8c",
    zIndex: 1,
  },
  barFirst: {
    borderTopLeftRadius: `${gaugeBarRadius}px`,
    borderBottomLeftRadius: `${gaugeBarRadius}px`,
  },
  barLast: {
    borderTopRightRadius: `${gaugeBarRadius}px`,
    borderBottomRightRadius: `${gaugeBarRadius}px`,
  },
  bg: {
    position: "absolute",
    width: "100%",
    height: "100%",
    left: 0,
    top: 0,
    zIndex: 0,
  },
};

const BatteryGauge = ({ drone_id, mode, percentage }) => {
  const percent10 = Math.round((percentage / 100) * gaugeBarsNb);
  const percentageArray = [...Array(percent10).keys()];
  console.log("TESTING" + String(drone_id));
  return (
    <>
      <div style={styles.container}>
        <img src={GaugeBg} style={styles.bg} />
        <div style={styles.barsContainer}>
          {percentageArray.map((ele, index) => (
            <div style={styles.barContainer}>
              {index === 0 ? (
                <div
                  key={index}
                  style={{ ...styles.bar, ...styles.barFirst }}
                />
              ) : index === gaugeBarsNb - 1 ? (
                <div key={index} style={{ ...styles.bar, ...styles.barLast }} />
              ) : (
                <div key={index} style={{ ...styles.bar }} />
              )}
            </div>
          ))}
        </div>
      </div>
      <div>DRONE ID - {drone_id}</div>
      <div>MODE - {mode}</div>
      <div>Percentage - {percentage}%</div>
    </>
  );
};

export default BatteryGauge;
