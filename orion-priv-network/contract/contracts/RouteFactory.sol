//SPDX-License-Identifier: MIT
pragma solidity ^0.8.9;

import "./RouteContract.sol";
import "@openzeppelin/contracts/utils/math/SafeMath.sol";

contract RouteFactory {
    using SafeMath for uint256;

    enum RouteNames {
        AP7,
        M40,
        CITY
    }

    enum RouteLocations {
        ALICANTE,
        ELCHE
    }

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

    function getRouteCars(
        uint256 _routeIndex
    ) public view returns (address[] memory) {
        return
            RouteContract(address(routes_addresses[_routeIndex]))
                .getWhiteListedCars();
    }

    function getRouteCarsNumber(
        uint256 _routeIndex
    ) public view returns (uint256) {
        return
            RouteContract(address(routes_addresses[_routeIndex]))
                .getWhiteListedCarsNumber();
    }

    function getRouteName(
        uint256 _routeIndex
    ) public view returns (string memory) {
        return RouteContract(routes_addresses[_routeIndex]).getName();
    }

    function getRouteIndex(
        string memory _location,
        string memory _name
    ) public view returns (uint256) {
        return s_routes_ids[_location][_name] - 1;
    }

    function getNextId() public view returns (uint256) {
        return s_nextRouteId;
    }

    function getNumberOfRoutes() public view returns (uint256) {
        return s_routeCounter;
    }
}
