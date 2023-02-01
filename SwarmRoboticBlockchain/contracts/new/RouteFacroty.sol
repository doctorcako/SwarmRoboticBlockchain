//SPDX-License-Identifier: MIT
pragma solidity ^0.8.9;

import "./RouteContract.sol";
import "@openzeppelin/contracts/utils/math/SafeMath.sol";

contract RouteFactory {
    using SafeMath for uint256;

    uint256 private s_nextRouteId = 1;
    uint256 private s_routeCounter = 0;
    RouteContract[] private routes_addresses;
    mapping(string => mapping(string => uint256)) private s_routes_ids;

    //create a modifier that checks if the route already exists
    modifier routeDoesntExists(string memory _location, string memory _name) {
        require(s_routes_ids[_location][_name] == 0, "Route already exists");
        _;
    }

    function createRoute(
        string memory _location,
        string memory _name
    ) public routeDoesntExists(_location, _name) {
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
        RouteContract(address(routes_addresses[_routeIndex])).enterWhiteList();
    }

    function leaveRoute(uint256 _routeIndex) public {
        RouteContract(address(routes_addresses[_routeIndex])).leaveWhiteList();
    }

    function gfSetter(uint256 _routeIndex, string memory _name) public {
        RouteContract(address(routes_addresses[_routeIndex])).setName(_name);
    }

    function getRouteName(
        uint256 _routeIndex
    ) public view returns (string memory) {
        return RouteContract(routes_addresses[_routeIndex]).getName();
    }

    function getRouteId(
        string memory _location,
        string memory _name
    ) public view returns (uint256) {
        return s_routes_ids[_location][_name];
    }

    function getNextId() public view returns (uint256) {
        return s_nextRouteId;
    }

    function getNumberOfRoutes() public view returns (uint256) {
        return s_routeCounter;
    }
}
