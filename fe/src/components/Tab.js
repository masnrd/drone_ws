import React from "react";
import PropTypes from "prop-types";

// This function determines the appropriate close icon based on the props.
const CloseIconSelector = ({ closeIcon, position }) => {
  return closeIcon;
};

CloseIconSelector.propTypes = {
  closeIcon: PropTypes.oneOfType([PropTypes.string, PropTypes.element]),
  position: PropTypes.oneOf(["left", "right"]),
};

const Tab = ({
  id,
  header,
  children,
  onClose,
  active,
  closeIcon,
  position,
}) => {
  const activeClass = active ? " active" : "";

  return (
    <div id={id} class="mt-30 p-20..." className={`sidebar-pane${activeClass}`}>
      <h1 className="sidebar-header">
        {header}
        <div
          className="sidebar-close"
          role="button" // Corrected 'btn' to 'button' for valid role value
          onClick={onClose}
        >
          <CloseIconSelector closeIcon={closeIcon} position={position} />
        </div>
      </h1>
      {children}
    </div>
  );
};

Tab.propTypes = {
  id: PropTypes.string.isRequired,
  header: PropTypes.string.isRequired,
  children: PropTypes.node.isRequired, // Simplified to `node` which covers functions, elements, and more
  onClose: PropTypes.func,
  active: PropTypes.bool,
  closeIcon: PropTypes.oneOfType([PropTypes.string, PropTypes.element]),
  position: PropTypes.oneOf(["left", "right"]),
};

export default Tab;
