import React, { useEffect, useRef } from "react";
import PropTypes from "prop-types";
import Tab from "./Tab";
import MenuButton from "./MenuButton";
import "./sidebar.scss";

// Adapted from https://github.com/eferhatg/react-leaflet-sidetabs/tree/b677e0b37c1fac8a86a421d059cb38a275b1024d
const breakpoints = [
  parseInt(
    getComputedStyle(document.documentElement).getPropertyValue(
      "--breakpoint-s"
    ),
    10
  ),
  parseInt(
    getComputedStyle(document.documentElement).getPropertyValue(
      "--breakpoint-m"
    ),
    10
  ),
  parseInt(
    getComputedStyle(document.documentElement).getPropertyValue(
      "--breakpoint-l"
    ),
    10
  ),
];

const widths = [
  parseInt(
    getComputedStyle(document.documentElement).getPropertyValue(
      "--leaflet-sidetabs-width-s"
    ),
    10
  ),
  parseInt(
    getComputedStyle(document.documentElement).getPropertyValue(
      "--leaflet-sidetabs-width-m"
    ),
    10
  ),
  parseInt(
    getComputedStyle(document.documentElement).getPropertyValue(
      "--leaflet-sidetabs-width-l"
    ),
    10
  ),
];

const TabType = PropTypes.shape({
  type: PropTypes.oneOf([Tab]),
});

const Sidebar = ({
  rehomeControls,
  position,
  onClose,
  onOpen,
  panMapOnChange,
  map,
  collapsed,
  selected,
  closeIcon,
  children,
  id,
}) => {
  const rootElement = useRef(null);

  useEffect(() => {
    if (rehomeControls) {
      const selector = `.leaflet-${position}`;
      const controls = document.querySelectorAll(selector);
      const topControl = document.querySelector(`.leaflet-top${selector}`);
      const bottomControl = document.querySelector(
        `.leaflet-bottom${selector}`
      );

      topControl.classList.add(`rehomed-top-${position}`);
      bottomControl.classList.add(`rehomed-bottom-${position}`);

      // Exception for Attribution control
      const attributionControl = document.querySelector(
        `${selector} .leaflet-control-attribution`
      );
      if (attributionControl) {
        const backupOriginalHome = document.createElement("div");
        const leafletControlContainer = document.querySelector(
          ".leaflet-control-container"
        );
        backupOriginalHome.classList.add(`leaflet-${position}`);
        backupOriginalHome.classList.add("leaflet-bottom");
        backupOriginalHome.appendChild(attributionControl);
        leafletControlContainer.appendChild(backupOriginalHome);
      }

      controls.forEach((control) => rootElement.current.appendChild(control));
    }
  }, [rehomeControls, position]); // Only re-run if these props change

  const handleOpen = (e, tabid) => {
    e.preventDefault();
    e.stopPropagation();
    if (onOpen) {
      onOpen(tabid);
    }
    if (panMapOnChange && collapsed) {
      if (map) {
        map.panBy([-getOffset() / 2, 0], { duration: 0.5 });
      } else {
        console.error(
          `react-leaflet-sidetabs: 'panMapOnChange' prop requires that 'map' prop is provided, 'map' prop not provided`
        );
      }
    }
  };

  const handleClose = (e) => {
    e.preventDefault();
    e.stopPropagation();
    if (onClose) {
      onClose(e);
    }
    if (panMapOnChange) {
      if (map) {
        map.panBy([getOffset() / 2, 0], { duration: 0.5 });
      } else {
        console.error(
          `react-leaflet-sidetabs: 'panMapOnChange' prop requires that 'map' prop is provided, 'map' prop not provided`
        );
      }
    }
  };

  const getOffset = () => {
    const windowSize = window.innerWidth;
    let offset = 0;
    for (let i = 0; i < breakpoints.length - 1; i++) {
      if (windowSize > breakpoints[i] && windowSize <= breakpoints[i + 1]) {
        offset = widths[i] / 2;
        break; // Add break to stop the loop once the condition is met
      }
    }
    if (windowSize > breakpoints[breakpoints.length - 1]) {
      offset = widths[widths.length - 1] / 2;
    }
    return position === "left" ? offset : -offset;
  };

  const renderPanes = (children) => {
    return React.Children.map(children, (p) =>
      React.cloneElement(p, {
        onClose: handleClose,
        closeIcon,
        active: p.props.id === selected,
        position: position || "left",
      })
    );
  };

  const positionClass = ` sidebar-${position || "left"}`;
  const collapsedClass = collapsed ? " collapsed" : "";
  const tabs = React.Children.toArray(children);
  const bottomTabs = tabs.filter((t) => t.props.anchor === "bottom");
  const topTabs = tabs.filter((t) => t.props.anchor !== "bottom");

  return (
    <div
      id={id || "leaflet-sidebar"}
      className={`sidebar leaflet-touch${positionClass}${collapsedClass}`}
      ref={rootElement}
    >
      <div className="sidebar-tabs">
        <ul role="tablist">
          {topTabs.map((t) => (
            <MenuButton
              key={t.props.id}
              id={t.props.id}
              icon={t.props.icon}
              disabled={t.props.disabled}
              selected={selected}
              collapsed={collapsed}
              onClose={handleClose}
              onOpen={handleOpen}
              map={map || null}
            />
          ))}
        </ul>
        <ul role="tablist">
          {bottomTabs.map((t) => (
            <MenuButton
              key={t.props.id}
              id={t.props.id}
              icon={t.props.icon}
              disabled={t.props.disabled}
              selected={selected}
              collapsed={collapsed}
              onClose={handleClose}
              onOpen={handleOpen}
              map={map || null}
            />
          ))}
        </ul>
      </div>
      <div className="sidebar-content">{renderPanes(children)}</div>
    </div>
  );
};

Sidebar.propTypes = {
  id: PropTypes.string,
  map: PropTypes.object,
  collapsed: PropTypes.bool.isRequired,
  position: PropTypes.oneOf(["left", "right"]),
  selected: PropTypes.oneOfType([PropTypes.string, PropTypes.bool]),
  closeIcon: PropTypes.oneOfType([PropTypes.string, PropTypes.element]),
  onClose: PropTypes.func,
  onOpen: PropTypes.func,
  panMapOnChange: PropTypes.bool,
  children: PropTypes.oneOfType([PropTypes.arrayOf(TabType), TabType]),
  rehomeControls: PropTypes.bool,
};

export default Sidebar;
