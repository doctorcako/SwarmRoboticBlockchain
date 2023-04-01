//SPDX-License-Identifier: MIT
pragma solidity ^0.8.9;

import "./RouteContract.sol";
import "@openzeppelin/contracts/utils/math/SafeMath.sol";

// Compile: solcjs --bin --abi --include-path node_modules/ --base-path . RouteFactory.sol

contract RouteFactory {
    using SafeMath for uint256;

    address private immutable i_admin;
    uint256 private s_nextRouteId;
    uint256 private s_routeCounter;
    RouteContract[] private routes_addresses;
    mapping(string => mapping(string => uint256)) private s_routes_ids;

    event routeDeleted(uint256 _deletedId);

    //set the constructor
    constructor() {
        s_nextRouteId = 1;
        s_routeCounter = 0;
        routes_addresses = new RouteContract[](0);
        i_admin = msg.sender;
    }

    //create a modifier that checks if the route already exists
    modifier approveRouteCreation(
        string memory _location,
        string memory _name
    ) {
        require(s_routes_ids[_location][_name] == 0, "Route already exists");
        _;
    }

    modifier routeExists(string memory _location, string memory _name) {
        require(s_routes_ids[_location][_name] != 0, "Route doesn't exists");
        _;
    }

    modifier isAdmin() {
        require(i_admin == msg.sender, "Not allowed to create routes");
        _;
    }

    function createRoute(
        string memory _location,
        string memory _name
    ) public isAdmin approveRouteCreation(_location, _name) {
        RouteContract route = new RouteContract(
            s_nextRouteId,
            _location,
            _name
        );
        s_routes_ids[_location][_name] = s_nextRouteId;
        routes_addresses.push(route);
        s_nextRouteId = s_nextRouteId.add(1);
        s_routeCounter = s_routeCounter.add(1);
    }

    function enterRoute(uint256 _routeIndex) public {
        RouteContract(address(routes_addresses[_routeIndex])).enterWhiteList(
            msg.sender
        );
    }

    function leaveRoute(uint256 _routeIndex) public {
        RouteContract(address(routes_addresses[_routeIndex])).leaveWhiteList(
            msg.sender
        );
    }

    function setRouteStatus(
        uint256 _routeIndex,
        string memory _status
    ) public {
        RouteContract(address(routes_addresses[_routeIndex])).setStatus(
            _status
        );
    }

    function getRouteCars(
        uint256 _routeIndex
    ) public view returns (address[] memory) {
        return
            RouteContract(address(routes_addresses[_routeIndex]))
                .getWhiteListedCars();
    }

    function getRouteName(
        uint256 _routeIndex
    ) public view returns (string memory) {
        return RouteContract(routes_addresses[_routeIndex]).getNameAndLocation();
    }

    function getRouteIndex(
        string memory _location,
        string memory _name
    ) public view returns (uint256) {
        return s_routes_ids[_location][_name] - 1;
    }

    function getRouteStatus(
        uint256 _routeIndex
    ) public view returns (string memory) {
        return RouteContract(routes_addresses[_routeIndex]).getStatus();
    }

    function getNextId() public view returns (uint256) {
        return s_nextRouteId;
    }

    function getRoutes() public view returns (RouteContract[] memory) {
        //call to each route and get the name and location
        return routes_addresses;
    }

    function getRoutesNames() public view returns (string[] memory) {
        //call to each route and get the name and location
        string[] memory routes_names = new string[](s_routeCounter);
        for (uint256 i = 0; i < s_routeCounter; i++) {
            routes_names[i] = RouteContract(routes_addresses[i]).getNameAndLocation();
        }
        return routes_names;
    }

}
