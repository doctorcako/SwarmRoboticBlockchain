//SPDX-License-Identifier: MIT
pragma solidity ^0.8.9;

import "@openzeppelin/contracts/utils/math/SafeMath.sol";
import "hardhat/console.sol";

contract RouteContract {
    using SafeMath for uint256;

    uint256 private s_id;
    string private s_name;
    string private s_location;
    address[] private s_whitelist;
    mapping(address => bool) private s_carHasVoted;

    modifier notInWhiteList(address _car) {
        require(getCarFromWhitelist(_car) == address(0), "Already in route");
        _;
    }

    constructor(uint256 _id, string memory _location, string memory _name) {
        s_id = _id;
        s_name = _name;
        s_location = _location;
        s_whitelist = new address[](0);
    }

    function getName() public view returns (string memory) {
        return s_name;
    }

    function getCarFromWhitelist(address _car) public view returns (address) {
        address carAddress = address(0);
        for (uint256 i = 0; i < s_whitelist.length; i++) {
            if (s_whitelist[i] == _car) {
                carAddress = s_whitelist[i];
                break;
            }
        }
        return carAddress;
    }

    function updateId(uint256 _idDeleted) public payable {
        if (_idDeleted < s_id) {
            s_id = s_id.sub(1);
        } else {
            s_id = s_id.add(1);
        }
    }

    function enterWhiteList(address _car) public payable notInWhiteList(_car) {
        s_whitelist.push(_car);
    }

    function leaveWhiteList(address _car) public {
        for (uint i = 0; i < s_whitelist.length; i++) {
            if (s_whitelist[i] == _car) {
                s_whitelist[i] = s_whitelist[s_whitelist.length - 1];
                reorderWhiteList(i);
                break;
            }
        }
    }

    function reorderWhiteList(uint index) public {
        for (uint i = index; i < s_whitelist.length - 1; i++) {
            s_whitelist[i] = s_whitelist[i + 1];
        }
        s_whitelist.pop();
    }

    function getWhiteListedCars() public view returns (address[] memory) {
        return s_whitelist;
    }

    function getWhiteListedCarsNumber() public view returns (uint256) {
        return s_whitelist.length;
    }

    function setName(string memory _name) public payable {
        s_name = _name;
    }
}
